using UnrealBuildTool;

public class A320Sim : ModuleRules
{
	public A320Sim(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
		// Each file keeps its own colour constants; unity blobs would make them shadow each other.
		bUseUnity = false;
		PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine", "InputCore" });
		PrivateDependencyModuleNames.AddRange(new string[] { "RenderCore", "A320Core", "ImageWrapper", "ProceduralMeshComponent" });
		if (Target.Platform == UnrealTargetPlatform.Win64)
		{
			// DirectInput: USB joysticks, throttle quadrants, pedals and FCU panels (128 buttons each).
			PublicSystemLibraries.AddRange(new string[] { "dinput8.lib", "dxguid.lib" });
			// USB HID: WingFlex FCU and EFIS Cube lights and displays.
			PublicSystemLibraries.AddRange(new string[] { "hid.lib", "setupapi.lib" });
			// SAPI text-to-speech for the ATC voices (COM).
			PublicSystemLibraries.AddRange(new string[] { "ole32.lib" });
		}
	}
}
