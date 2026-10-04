#include "A320WingFlex.h"

#include "A320Aircraft.h"
#include "A320Commands.h"
#include "A320Sim.h"
#include "HAL/PlatformTime.h"

#if PLATFORM_WINDOWS
#include "Windows/WindowsHWrapper.h"
#include "Windows/AllowWindowsPlatformTypes.h"
#include <setupapi.h>
extern "C"
{
#include <hidsdi.h>
}
#endif

using namespace a320::wingflex;

// One open panel: overlapped reads and writes, so the game thread never waits on USB.
struct FA320HidLink
{
#if PLATFORM_WINDOWS
	HANDLE Handle = INVALID_HANDLE_VALUE;
	OVERLAPPED ReadOv = {};
	OVERLAPPED WriteOv = {};
#endif
	TArray<uint8> ReadBuf;
	TArray<uint8> WriteBuf;
	bool bReadPending = false;
	bool bWritePending = false;
	bool bFailed = false;

	~FA320HidLink()
	{
#if PLATFORM_WINDOWS
		if (Handle != INVALID_HANDLE_VALUE)
		{
			CancelIo(Handle);
			CloseHandle(Handle);
		}
		if (ReadOv.hEvent)
		{
			CloseHandle(ReadOv.hEvent);
		}
		if (WriteOv.hEvent)
		{
			CloseHandle(WriteOv.hEvent);
		}
#endif
	}

	// The next input report's payload (without the report ID), if one has arrived.
	bool Read(TArray<uint8>& OutPayload)
	{
#if PLATFORM_WINDOWS
		DWORD Bytes = 0;
		if (!bReadPending)
		{
			ResetEvent(ReadOv.hEvent);
			if (ReadFile(Handle, ReadBuf.GetData(), static_cast<DWORD>(ReadBuf.Num()), &Bytes, &ReadOv))
			{
				return TakeReport(Bytes, OutPayload);
			}
			if (GetLastError() != ERROR_IO_PENDING)
			{
				bFailed = true;
				return false;
			}
			bReadPending = true;
		}
		if (GetOverlappedResult(Handle, &ReadOv, &Bytes, 0))
		{
			bReadPending = false;
			return TakeReport(Bytes, OutPayload);
		}
		if (GetLastError() != ERROR_IO_INCOMPLETE)
		{
			bReadPending = false;
			bFailed = true;
		}
#endif
		return false;
	}

	bool TakeReport(uint32 Bytes, TArray<uint8>& OutPayload)
	{
		// Byte 0 is the report ID (0).
		if (Bytes < 2)
		{
			return false;
		}
		OutPayload.Reset();
		OutPayload.Append(ReadBuf.GetData() + 1, static_cast<int32>(Bytes) - 1);
		return true;
	}

	// Sends an output report unless the previous one is still on its way.
	bool Write(const Payload& Data)
	{
#if PLATFORM_WINDOWS
		if (bWritePending)
		{
			DWORD Bytes = 0;
			if (!GetOverlappedResult(Handle, &WriteOv, &Bytes, 0))
			{
				if (GetLastError() == ERROR_IO_INCOMPLETE)
				{
					return false;
				}
				bFailed = true;
			}
			bWritePending = false;
		}
		FMemory::Memzero(WriteBuf.GetData(), WriteBuf.Num());
		FMemory::Memcpy(WriteBuf.GetData() + 1, Data.data(), FMath::Min<int32>(static_cast<int32>(Data.size()), WriteBuf.Num() - 1));
		ResetEvent(WriteOv.hEvent);
		if (!WriteFile(Handle, WriteBuf.GetData(), static_cast<DWORD>(WriteBuf.Num()), nullptr, &WriteOv))
		{
			if (GetLastError() != ERROR_IO_PENDING)
			{
				bFailed = true;
				return false;
			}
			bWritePending = true;
		}
		return true;
#else
		return false;
#endif
	}
};

namespace
{
#if PLATFORM_WINDOWS
	// Opens the panel's HID collection that carries the 64-byte reports.
	TUniquePtr<FA320HidLink> OpenPanel(uint16 VendorId, uint16 ProductId)
	{
		GUID HidGuid;
		HidD_GetHidGuid(&HidGuid);
		HDEVINFO Info = SetupDiGetClassDevsW(&HidGuid, nullptr, nullptr, DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);
		if (Info == INVALID_HANDLE_VALUE)
		{
			return nullptr;
		}
		TUniquePtr<FA320HidLink> Found;
		SP_DEVICE_INTERFACE_DATA Interface = {};
		Interface.cbSize = sizeof(Interface);
		for (DWORD Index = 0; !Found && SetupDiEnumDeviceInterfaces(Info, nullptr, &HidGuid, Index, &Interface); ++Index)
		{
			DWORD Size = 0;
			SetupDiGetDeviceInterfaceDetailW(Info, &Interface, nullptr, 0, &Size, nullptr);
			if (Size == 0)
			{
				continue;
			}
			TArray<uint8> DetailBuf;
			DetailBuf.SetNumZeroed(static_cast<int32>(Size));
			SP_DEVICE_INTERFACE_DETAIL_DATA_W* Detail = reinterpret_cast<SP_DEVICE_INTERFACE_DETAIL_DATA_W*>(DetailBuf.GetData());
			Detail->cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA_W);
			if (!SetupDiGetDeviceInterfaceDetailW(Info, &Interface, Detail, Size, nullptr, nullptr))
			{
				continue;
			}
			HANDLE Handle = CreateFileW(Detail->DevicePath, GENERIC_READ | GENERIC_WRITE, FILE_SHARE_READ | FILE_SHARE_WRITE,
				nullptr, OPEN_EXISTING, FILE_FLAG_OVERLAPPED, nullptr);
			if (Handle == INVALID_HANDLE_VALUE)
			{
				continue;
			}
			HIDD_ATTRIBUTES Attributes = {};
			Attributes.Size = sizeof(Attributes);
			HIDP_CAPS Caps = {};
			PHIDP_PREPARSED_DATA Preparsed = nullptr;
			bool bMatch = HidD_GetAttributes(Handle, &Attributes) && Attributes.VendorID == VendorId && Attributes.ProductID == ProductId;
			if (bMatch && HidD_GetPreparsedData(Handle, &Preparsed))
			{
				bMatch = HidP_GetCaps(Preparsed, &Caps) == HIDP_STATUS_SUCCESS && Caps.InputReportByteLength >= 17 &&
					Caps.OutputReportByteLength >= 23;
				HidD_FreePreparsedData(Preparsed);
			}
			else
			{
				bMatch = false;
			}
			if (!bMatch)
			{
				CloseHandle(Handle);
				continue;
			}
			Found = MakeUnique<FA320HidLink>();
			Found->Handle = Handle;
			Found->ReadOv.hEvent = CreateEventW(nullptr, 1, 0, nullptr);
			Found->WriteOv.hEvent = CreateEventW(nullptr, 1, 0, nullptr);
			Found->ReadBuf.SetNumZeroed(Caps.InputReportByteLength);
			Found->WriteBuf.SetNumZeroed(Caps.OutputReportByteLength);
		}
		SetupDiDestroyDeviceInfoList(Info);
		return Found;
	}
#else
	TUniquePtr<FA320HidLink> OpenPanel(uint16, uint16) { return nullptr; }
#endif

	bool Pressed(bool Now, bool Before) { return Now && !Before; }
}

FA320WingFlex::FA320WingFlex() = default;

FA320WingFlex::~FA320WingFlex() = default;

bool FA320WingFlex::HasFcu() const
{
	return Fcu.IsValid();
}

bool FA320WingFlex::HasEfis() const
{
	return Efis.IsValid();
}

void FA320WingFlex::Scan()
{
	// A panel that is already open is left alone: only a newly opened one gets its full state.
	if (!Fcu)
	{
		Fcu = OpenPanel(kFcuVendorId, kFcuProductId);
		bHaveFcuInput = false;
		LastFcuOut = {};
		if (Fcu)
		{
			UE_LOG(LogA320, Log, TEXT("WingFlex FCU Cube connected"));
		}
	}
	if (!Efis)
	{
		Efis = OpenPanel(kEfisVendorId, kEfisProductId);
		bHaveEfisInput = false;
		LastEfisOut = {};
		if (Efis)
		{
			UE_LOG(LogA320, Log, TEXT("WingFlex EFIS Cube connected"));
		}
	}
}

void FA320WingFlex::Tick(AA320Aircraft& Aircraft, float DeltaSeconds)
{
	// Panels plugged in later are found within seconds. Looking through every USB device takes
	// a moment, so once one panel is there the other (often not owned) is looked for rarely.
	ScanTimer -= DeltaSeconds;
	if ((!Fcu || !Efis) && ScanTimer <= 0.0f)
	{
		ScanTimer = Fcu || Efis ? 30.0f : 3.0f;
		Scan();
	}
	TArray<uint8> Report;
	for (int32 i = 0; Fcu && i < 32 && Fcu->Read(Report); ++i)
	{
		FcuInput In;
		if (parseFcu(Report.GetData(), static_cast<size_t>(Report.Num()), In))
		{
			HandleFcu(Aircraft, In);
		}
	}
	for (int32 i = 0; Efis && i < 32 && Efis->Read(Report); ++i)
	{
		EfisInput In;
		if (parseEfis(Report.GetData(), static_cast<size_t>(Report.Num()), In))
		{
			HandleEfis(Aircraft, In);
		}
	}
	SendOutputs(Aircraft, FPlatformTime::Seconds());
	// Unplugged: drop it and look for it again.
	if (Fcu && Fcu->bFailed)
	{
		UE_LOG(LogA320, Log, TEXT("WingFlex FCU Cube disconnected"));
		Fcu.Reset();
	}
	if (Efis && Efis->bFailed)
	{
		UE_LOG(LogA320, Log, TEXT("WingFlex EFIS Cube disconnected"));
		Efis.Reset();
	}
}

void FA320WingFlex::HandleFcu(AA320Aircraft& Aircraft, const FcuInput& In)
{
	Backlight = steadyBrightness(Backlight, In.backlight);
	LcdBrightness = steadyBrightness(LcdBrightness, In.lcd);
	if (!bHaveFcuInput)
	{
		// The first report is the starting state, not a press.
		LastFcu = In;
		bHaveFcuInput = true;
		return;
	}
	struct FPress
	{
		FcuButton Button;
		EA320Command Command;
	};
	static const FPress Presses[] = {
		{kAp1, EA320Command::FcuAp},
		{kAp2, EA320Command::FcuAp2},
		{kAthr, EA320Command::FcuAthr},
		{kLoc, EA320Command::FcuLoc},
		{kAppr, EA320Command::FcuAppr},
		{kHdgPull, EA320Command::FcuHdgPull},
		{kHdgPush, EA320Command::FcuHdgPush},
		{kHdgTrk, EA320Command::FcuTrkFpa},
		{kAltPull, EA320Command::FcuAltPull},
		{kAltPush, EA320Command::FcuAltPush},
		{kVsPull, EA320Command::FcuVsPull},
		{kVsPush, EA320Command::FcuVsPush},
	};
	for (const FPress& P : Presses)
	{
		if (Pressed(In.button[P.Button], LastFcu.button[P.Button]))
		{
			Aircraft.ExecuteCommand(P.Command);
		}
	}
	// Knobs report the clicks turned since the last report. ALT steps 100 or 1000 ft with
	// the 100/1000 switch.
	struct FKnob
	{
		FcuKnob Knob;
		EA320Command Dec, Inc;
	};
	static const FKnob Knobs[] = {
		{kSpdKnob, EA320Command::SpdDec, EA320Command::SpdInc},
		{kHdgKnob, EA320Command::HdgDec, EA320Command::HdgInc},
		{kAltKnob, EA320Command::AltDec, EA320Command::AltInc},
		{kVsKnob, EA320Command::VsDec, EA320Command::VsInc},
	};
	for (const FKnob& K : Knobs)
	{
		const int32 Clicks = In.knob[K.Knob];
		const bool bLarge = K.Knob == kAltKnob && In.button[kAlt1000];
		for (int32 c = 0; c < FMath::Abs(Clicks); ++c)
		{
			Aircraft.ExecuteCommand(Clicks > 0 ? K.Inc : K.Dec, bLarge);
		}
	}
	LastFcu = In;
}

void FA320WingFlex::HandleEfis(AA320Aircraft& Aircraft, const EfisInput& In)
{
	// Selectors are synced to where the knobs point, also on the first report.
	struct FSelect
	{
		EfisButton Button;
		EA320Switch Switch;
		int32 Value;
	};
	static const FSelect Selects[] = {
		{kNdLs, EA320Switch::NdMode, A320_ND_ROSE_LS},
		{kNdVor, EA320Switch::NdMode, A320_ND_ROSE_NAV},
		{kNdNav, EA320Switch::NdMode, A320_ND_ROSE_NAV},
		{kNdArc, EA320Switch::NdMode, A320_ND_ARC},
		{kNdPlan, EA320Switch::NdMode, A320_ND_ROSE_NAV},
		{kRange10, EA320Switch::NdRange, 10},
		{kRange20, EA320Switch::NdRange, 20},
		{kRange40, EA320Switch::NdRange, 40},
		{kRange80, EA320Switch::NdRange, 80},
		{kRange160, EA320Switch::NdRange, 160},
		{kRange320, EA320Switch::NdRange, 320},
	};
	for (const FSelect& S : Selects)
	{
		if (In.button[S.Button] && (!bHaveEfisInput || !LastEfis.button[S.Button]))
		{
			Aircraft.SetSwitch(S.Switch, S.Value);
		}
	}
	if (bHaveEfisInput)
	{
		if (Pressed(In.button[kLs], LastEfis.button[kLs]))
		{
			Aircraft.ExecuteCommand(EA320Command::LsToggle);
		}
		if (Pressed(In.button[kMasterWarn], LastEfis.button[kMasterWarn]) || Pressed(In.button[kMasterCaution], LastEfis.button[kMasterCaution]))
		{
			Aircraft.ExecuteCommand(EA320Command::MasterWarnAck);
		}
	}
	LastEfis = In;
	bHaveEfisInput = true;
}

void FA320WingFlex::SendOutputs(const AA320Aircraft& Aircraft, double Now)
{
	const A320State& St = Aircraft.GetSimState();
	const bool bFlash = FMath::Fmod(Now, 0.8) < 0.5;
	if (Fcu)
	{
		// The same lights and windows as the FCU strip on screen.
		FcuOutput O;
		const bool bAppr = (St.armed & A320_ARMED_GS) || St.vertMode == A320_VERT_GS_STAR || St.vertMode == A320_VERT_GS ||
			St.vertMode == A320_VERT_LAND || St.vertMode == A320_VERT_FLARE;
		O.appr = bAppr;
		O.loc = !bAppr && (St.latMode == A320_LAT_LOC || St.latMode == A320_LAT_LOC_STAR || (St.armed & A320_ARMED_LOC));
		O.ap1 = St.ap1Engaged != 0;
		O.ap2 = St.ap2Engaged != 0;
		O.athr = St.athrEngaged != 0;
		// On the localizer the heading window shows dashes; out of V/S mode the V/S window does.
		// NAV (engaged or armed): dashes and the managed dot.
		const bool bNav = St.latMode == A320_LAT_NAV || (St.armed & A320_ARMED_NAV);
		O.hdgDashed = bNav || St.latMode == A320_LAT_LOC_STAR || St.latMode == A320_LAT_LOC || St.latMode == A320_LAT_ROLLOUT;
		O.hdgManaged = bNav;
		O.altManaged = St.vertMode == A320_VERT_CLB || St.vertMode == A320_VERT_DES || St.vertMode == A320_VERT_ALT_CST ||
			St.vertMode == A320_VERT_ALT_CST_STAR || (St.armed & A320_ARMED_CLB);
		O.vsDashed = St.vertMode != A320_VERT_VS && St.vertMode != A320_VERT_FPA;
		O.trkFpa = St.fcuTrkFpa != 0;
		O.spd = static_cast<uint16>(FMath::Clamp(FMath::RoundToInt(St.fcuSpdKt), 0, 999));
		O.hdg = static_cast<uint16>((FMath::RoundToInt(St.fcuHdgMagDeg) + 359) % 360 + 1);
		O.alt = static_cast<uint16>(FMath::Clamp(FMath::RoundToInt(St.fcuAltFt), 0, 65000));
		// FPA in tenths of a degree (-2.5 as -25). MobiFlight's driver doesn't document the FPA
		// format; this assumes the panel adds the point itself in TRK-FPA.
		O.vs = static_cast<int16>(St.fcuTrkFpa ? FMath::RoundToInt(St.fcuFpaDeg * 10.0)
			: FMath::Clamp(FMath::RoundToInt(St.fcuVsFpm), -9900, 9900));
		O.backlight = Backlight;
		O.lcd = LcdBrightness;
		const Payload Out = buildFcu(O);
		// Changes go out at once; the unchanged state is repeated as a keep-alive, since the panels
		// dim their lights when the host goes quiet (MobiFlight streams continuously).
		if ((Out != LastFcuOut || Now - LastFcuSendTime > KeepAliveS) && Fcu->Write(Out))
		{
			LastFcuOut = Out;
			LastFcuSendTime = Now;
		}
	}
	if (Efis)
	{
		EfisOutput O;
		const uint32 Warnings = St.warnings & ~uint32(A320_WARN_CAUTION_MASK);
		const uint32 Cautions = St.warnings & uint32(A320_WARN_CAUTION_MASK);
		O.masterWarn = Warnings != 0 && bFlash;
		O.masterCaution = Cautions != 0 && bFlash;
		O.ls = Aircraft.IsLsOn();
		O.backlight = Backlight;
		O.lcd = LcdBrightness;
		const Payload Out = buildEfis(O);
		if ((Out != LastEfisOut || Now - LastEfisSendTime > KeepAliveS) && Efis->Write(Out))
		{
			LastEfisOut = Out;
			LastEfisSendTime = Now;
		}
	}
}

#if PLATFORM_WINDOWS
#include "Windows/HideWindowsPlatformTypes.h"
#endif
