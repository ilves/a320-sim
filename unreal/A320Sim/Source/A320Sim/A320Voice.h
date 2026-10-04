#pragma once

#include "CoreMinimal.h"
#include "Containers/Queue.h"
#include "HAL/Runnable.h"

#include <atomic>

class FEvent;
class FRunnableThread;

// Speaks the radio messages with the Windows voices on a worker thread (speech synthesis takes
// longer than a frame); the PCM comes back to the game thread for the radio effect.
class FA320Voice : public FRunnable
{
public:
	FA320Voice();
	virtual ~FA320Voice() override;

	// Speaker: A320AtcSpeaker. The clip later plays only while COM 1 is on FrequencyKhz.
	void Speak(const FString& Text, int32 Speaker, int32 FrequencyKhz);
	// A finished clip (game thread); false when none is ready.
	bool PopClip(TArray<int16>& OutPcm, int32& OutFrequencyKhz);
	// Drops everything queued or being spoken (a new flight).
	void Flush()
	{
		++Generation;
		bCancelSpeech = true;
	}
	// Installed voices: -1 while starting, 0 when there are none (text only).
	int32 GetVoiceCount() const { return VoiceCount.load(); }
	static constexpr int32 SampleRate = 22050;

	virtual uint32 Run() override;
	virtual void Stop() override;

private:
	struct FJob
	{
		FString Text;
		int32 Speaker = 0;
		int32 FrequencyKhz = 0;
		uint32 Generation = 0;
	};
	struct FClip
	{
		TArray<int16> Pcm;
		int32 FrequencyKhz = 0;
		uint32 Generation = 0;
	};

	TQueue<FJob, EQueueMode::Spsc> Jobs;
	TQueue<FClip, EQueueMode::Spsc> Clips;
	FEvent* Wake = nullptr;
	FRunnableThread* Thread = nullptr;
	std::atomic<bool> bStopping{false};
	std::atomic<int32> VoiceCount{-1};
	// Bumped by Flush(); jobs and clips from an older generation are skipped (both queues are
	// single-consumer, so neither side can drain the other's).
	std::atomic<uint32> Generation{0};
	std::atomic<bool> bCancelSpeech{false};  // stops the transmission being synthesised
};
