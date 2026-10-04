#include "A320Speech.h"

#include "CoreMinimal.h"

#if PLATFORM_WINDOWS
#include "Windows/WindowsHWrapper.h"
#include "Windows/AllowWindowsPlatformTypes.h"
THIRD_PARTY_INCLUDES_START
#include <objbase.h>
#include <mmreg.h>
#include <sapi.h>
THIRD_PARTY_INCLUDES_END
#include "Windows/HideWindowsPlatformTypes.h"
#endif

namespace a320speech
{
#if PLATFORM_WINDOWS
	struct Synthesizer::FImpl
	{
		bool bComReady = false;
		bool bOwnsCom = false;
		ISpVoice* Voice = nullptr;
		std::vector<ISpObjectToken*> Tokens;
		std::vector<std::wstring> Names;
		int AtcVoice = 0, CrewVoice = 0, AtisVoice = 0;
	};

	namespace
	{
		// SPDFID_WaveFormatEx, defined here: not every SDK's import libraries carry it.
		const GUID kWaveFormatEx = {0xc31adbae, 0x527f, 0x4ff5, {0xa2, 0x30, 0xf6, 0x2b, 0xb6, 0x1f, 0xf7, 0x0c}};

		std::wstring TokenString(ISpObjectToken* Token, const wchar_t* SubKey, const wchar_t* Value)
		{
			std::wstring Result;
			ISpDataKey* Key = nullptr;
			wchar_t* Text = nullptr;
			if (SubKey)
			{
				if (FAILED(Token->OpenKey(SubKey, &Key)) || !Key)
				{
					return Result;
				}
				if (SUCCEEDED(Key->GetStringValue(Value, &Text)) && Text)
				{
					Result = Text;
					CoTaskMemFree(Text);
				}
				Key->Release();
				return Result;
			}
			if (SUCCEEDED(Token->GetStringValue(Value, &Text)) && Text)
			{
				Result = Text;
				CoTaskMemFree(Text);
			}
			return Result;
		}
	}

	Synthesizer::Synthesizer()
		: Impl(new FImpl)
	{
		// Already initialised in another mode (RPC_E_CHANGED_MODE) is usable too, but not ours to uninitialise.
		const HRESULT Init = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
		Impl->bOwnsCom = SUCCEEDED(Init);
		Impl->bComReady = Impl->bOwnsCom || Init == RPC_E_CHANGED_MODE;
		if (!Impl->bComReady)
		{
			return;
		}
		if (FAILED(CoCreateInstance(__uuidof(SpVoice), nullptr, CLSCTX_ALL, __uuidof(ISpVoice), reinterpret_cast<void**>(&Impl->Voice))))
		{
			Impl->Voice = nullptr;
			return;
		}
		// All installed voices; English ones are preferred, since the phraseology is English.
		ISpObjectTokenCategory* Category = nullptr;
		IEnumSpObjectTokens* Enum = nullptr;
		if (SUCCEEDED(CoCreateInstance(__uuidof(SpObjectTokenCategory), nullptr, CLSCTX_ALL,
				__uuidof(ISpObjectTokenCategory),
				reinterpret_cast<void**>(&Category))) &&
			SUCCEEDED(Category->SetId(SPCAT_VOICES, false)) && SUCCEEDED(Category->EnumTokens(nullptr, nullptr, &Enum)))
		{
			ISpObjectToken* Token = nullptr;
			while (Enum->Next(1, &Token, nullptr) == S_OK && Token)
			{
				Impl->Tokens.push_back(Token);
				Impl->Names.push_back(TokenString(Token, nullptr, nullptr));
				Token = nullptr;
			}
		}
		if (Enum)
		{
			Enum->Release();
		}
		if (Category)
		{
			Category->Release();
		}
		int FirstEnglish = -1, Male = -1, Female = -1, Other = -1;
		for (int i = 0; i < static_cast<int>(Impl->Tokens.size()); ++i)
		{
			const std::wstring Language = TokenString(Impl->Tokens[i], L"Attributes", L"Language");
			// Language ids are hex lists; 409 is US English, 809 British English.
			const bool bEnglish = Language.find(L"409") != std::wstring::npos || Language.find(L"809") != std::wstring::npos ||
				Impl->Names[i].find(L"English") != std::wstring::npos;
			if (!bEnglish)
			{
				continue;
			}
			const std::wstring Gender = TokenString(Impl->Tokens[i], L"Attributes", L"Gender");
			if (FirstEnglish < 0)
			{
				FirstEnglish = i;
			}
			if (Gender == L"Male" && Male < 0)
			{
				Male = i;
			}
			else if (Gender == L"Female" && Female < 0)
			{
				Female = i;
			}
			else if (Other < 0 && i != Male && i != Female)
			{
				Other = i;
			}
		}
		const int Fallback = FirstEnglish >= 0 ? FirstEnglish : 0;
		Impl->AtcVoice = Male >= 0 ? Male : Fallback;
		Impl->CrewVoice = Female >= 0 && Female != Impl->AtcVoice ? Female : (Other >= 0 ? Other : Fallback);
		Impl->AtisVoice = Female >= 0 ? Female : Fallback;
	}

	Synthesizer::~Synthesizer()
	{
		for (ISpObjectToken* Token : Impl->Tokens)
		{
			Token->Release();
		}
		if (Impl->Voice)
		{
			Impl->Voice->Release();
		}
		if (Impl->bOwnsCom)
		{
			CoUninitialize();
		}
		delete Impl;
	}

	bool Synthesizer::IsReady() const { return Impl->Voice != nullptr; }

	int Synthesizer::VoiceCount() const { return static_cast<int>(Impl->Tokens.size()); }

	std::wstring Synthesizer::VoiceName(int Index) const
	{
		return Index >= 0 && Index < static_cast<int>(Impl->Names.size()) ? Impl->Names[static_cast<size_t>(Index)] : std::wstring();
	}

	bool Synthesizer::Speak(const std::wstring& Text, Speaker Who, const std::atomic<bool>& Cancel, std::vector<int16_t>& OutPcm)
	{
		OutPcm.clear();
		if (!Impl->Voice || Text.empty())
		{
			return false;
		}
		IStream* Memory = nullptr;
		if (FAILED(CreateStreamOnHGlobal(nullptr, TRUE, &Memory)) || !Memory)
		{
			return false;
		}
		ISpStream* Stream = nullptr;
		HRESULT Result = CoCreateInstance(__uuidof(SpStream), nullptr, CLSCTX_ALL, __uuidof(ISpStream), reinterpret_cast<void**>(&Stream));
		WAVEFORMATEX Format = {};
		Format.wFormatTag = 1;  // WAVE_FORMAT_PCM
		Format.nChannels = 1;
		Format.nSamplesPerSec = kSampleRate;
		Format.wBitsPerSample = 16;
		Format.nBlockAlign = 2;
		Format.nAvgBytesPerSec = kSampleRate * 2;
		if (SUCCEEDED(Result))
		{
			Result = Stream->SetBaseStream(Memory, kWaveFormatEx, &Format);
		}
		const int VoiceIndex = Who == Speaker::Atc ? Impl->AtcVoice : (Who == Speaker::Crew ? Impl->CrewVoice : Impl->AtisVoice);
		if (SUCCEEDED(Result) && VoiceIndex < static_cast<int>(Impl->Tokens.size()))
		{
			Impl->Voice->SetVoice(Impl->Tokens[static_cast<size_t>(VoiceIndex)]);
		}
		// Controllers talk briskly; the ATIS is read a little slower.
		Impl->Voice->SetRate(Who == Speaker::Atis ? 0 : 2);
		// false: SAPI converts to our format instead of switching the stream to the voice's own rate.
		if (SUCCEEDED(Result))
		{
			Result = Impl->Voice->SetOutput(Stream, false);
		}
		if (SUCCEEDED(Result))
		{
			Result = Impl->Voice->Speak(Text.c_str(), SPF_ASYNC | SPF_IS_NOT_XML, nullptr);
		}
		bool bCancelled = false;
		while (SUCCEEDED(Result) && Impl->Voice->WaitUntilDone(50) == S_FALSE)
		{
			if (Cancel.load())
			{
				Impl->Voice->Speak(nullptr, SPF_PURGEBEFORESPEAK, nullptr);
				bCancelled = true;
				break;
			}
		}
		Impl->Voice->SetOutput(nullptr, false);
		if (bCancelled)
		{
			Result = E_ABORT;
		}
		if (SUCCEEDED(Result))
		{
			STATSTG Stat = {};
			Memory->Stat(&Stat, STATFLAG_NONAME);
			const ULONG Bytes = static_cast<ULONG>(Stat.cbSize.QuadPart);
			std::vector<uint8_t> Raw(Bytes);
			LARGE_INTEGER Zero = {};
			Memory->Seek(Zero, STREAM_SEEK_SET, nullptr);
			ULONG Read = 0;
			if (Bytes > 0)
			{
				Memory->Read(Raw.data(), Bytes, &Read);
			}
			// Raw PCM is expected; skip a WAV header if one was written anyway.
			size_t Offset = Read >= 44 && Raw[0] == 'R' && Raw[1] == 'I' && Raw[2] == 'F' && Raw[3] == 'F' ? 44 : 0;
			OutPcm.resize((Read - Offset) / 2);
			for (size_t i = 0; i < OutPcm.size(); ++i)
			{
				OutPcm[i] = static_cast<int16_t>(Raw[Offset + 2 * i] | (Raw[Offset + 2 * i + 1] << 8));
			}
		}
		if (Stream)
		{
			Stream->Release();
		}
		Memory->Release();
		return SUCCEEDED(Result) && !OutPcm.empty();
	}
#else
	struct Synthesizer::FImpl
	{
	};
	Synthesizer::Synthesizer() : Impl(new FImpl) {}
	Synthesizer::~Synthesizer() { delete Impl; }
	bool Synthesizer::IsReady() const { return false; }
	int Synthesizer::VoiceCount() const { return 0; }
	std::wstring Synthesizer::VoiceName(int) const { return std::wstring(); }
	bool Synthesizer::Speak(const std::wstring&, Speaker, const std::atomic<bool>&, std::vector<int16_t>& OutPcm)
	{
		OutPcm.clear();
		return false;
	}
#endif
}
