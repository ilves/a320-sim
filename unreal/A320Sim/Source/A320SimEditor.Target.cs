using UnrealBuildTool;

public class A320SimEditorTarget : TargetRules
{
	public A320SimEditorTarget(TargetInfo Target) : base(Target)
	{
		Type = TargetType.Editor;
		DefaultBuildSettings = BuildSettingsVersion.Latest;
		IncludeOrderVersion = EngineIncludeOrderVersion.Latest;
		ExtraModuleNames.Add("A320Sim");
	}
}
