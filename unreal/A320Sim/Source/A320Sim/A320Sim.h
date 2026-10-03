#pragma once

#include "CoreMinimal.h"

DECLARE_LOG_CATEGORY_EXTERN(LogA320, Log, All);

// Not "A320Sim": that name is the C API's simulation handle type.
namespace A320CoreDll
{
	// True once A320Core.dll is loaded; nothing may call the a320_* API before that.
	bool IsLoaded();
}
