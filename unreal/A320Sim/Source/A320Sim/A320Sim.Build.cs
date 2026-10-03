using UnrealBuildTool;

public class A320Sim : ModuleRules
{
	public A320Sim(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
		// Each file keeps its own colour constants; unity blobs would make them shadow each other.
		bUseUnity = false;
		PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine", "InputCore" });
		PrivateDependencyModuleNames.AddRange(new string[] { "RenderCore", "A320Core" });
	}
}
