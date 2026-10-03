#include "A320Sim.h"

#include "HAL/PlatformProcess.h"
#include "Misc/Paths.h"
#include "Modules/ModuleManager.h"

DEFINE_LOG_CATEGORY(LogA320);

namespace
{
	void* GCoreDllHandle = nullptr;
}

bool A320CoreDll::IsLoaded()
{
	return GCoreDllHandle != nullptr;
}

class FA320SimModule : public FDefaultGameModuleImpl
{
public:
	virtual void StartupModule() override
	{
		// The DLL is delay-loaded; load it by full path so the editor (whose exe lives in the
		// engine folder) and packaged builds both find it next to the project binaries.
		const FString Candidates[] = {
			FPaths::Combine(FPaths::ProjectDir(), TEXT("Binaries/Win64/A320Core.dll")),
			FPaths::Combine(FPlatformProcess::BaseDir(), TEXT("A320Core.dll")),
		};
		for (const FString& Path : Candidates)
		{
			GCoreDllHandle = FPlatformProcess::GetDllHandle(*FPaths::ConvertRelativePathToFull(Path));
			if (GCoreDllHandle)
			{
				UE_LOG(LogA320, Log, TEXT("Loaded %s"), *Path);
				return;
			}
		}
		UE_LOG(LogA320, Error, TEXT("A320Core.dll not found. Run Setup.bat to build it."));
	}

	virtual void ShutdownModule() override
	{
		if (GCoreDllHandle)
		{
			FPlatformProcess::FreeDllHandle(GCoreDllHandle);
			GCoreDllHandle = nullptr;
		}
	}
};

IMPLEMENT_PRIMARY_GAME_MODULE(FA320SimModule, A320Sim, "A320Sim");
