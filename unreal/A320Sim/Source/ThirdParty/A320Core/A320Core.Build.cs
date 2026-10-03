using System.IO;
using UnrealBuildTool;

// The flight model (JSBSim + the A320 core) prebuilt by Setup.bat into this folder.
public class A320Core : ModuleRules
{
	public A320Core(ReadOnlyTargetRules Target) : base(Target)
	{
		Type = ModuleType.External;
		PublicSystemIncludePaths.Add(Path.Combine(ModuleDirectory, "include"));

		if (Target.Platform != UnrealTargetPlatform.Win64)
		{
			throw new BuildException("A320Core is only built for Win64 so far.");
		}

		string Dll = Path.Combine(ModuleDirectory, "bin", "A320Core.dll");
		string Lib = Path.Combine(ModuleDirectory, "lib", "A320Core.lib");
		if (!File.Exists(Dll) || !File.Exists(Lib))
		{
			throw new BuildException("A320Core.dll/.lib not found in " + ModuleDirectory + ". Run Setup.bat first.");
		}

		PublicAdditionalLibraries.Add(Lib);
		// Delay-loaded and loaded by full path in FA320SimModule::StartupModule.
		PublicDelayLoadDLLs.Add("A320Core.dll");
		RuntimeDependencies.Add("$(BinaryOutputDir)/A320Core.dll", Dll);
	}
}
