#include "A320Voice.h"

#include "A320Sim.h"
#include "A320Speech.h"
#include "HAL/Event.h"
#include "HAL/PlatformProcess.h"
#include "HAL/RunnableThread.h"
#include "a320/a320_api.h"

#include <string>
#include <vector>

static_assert(FA320Voice::SampleRate == a320speech::kSampleRate, "the radio clip rate must match the synthesizer's");

FA320Voice::FA320Voice()
{
	Wake = FPlatformProcess::GetSynchEventFromPool(false);
	Thread = FRunnableThread::Create(this, TEXT("A320Voice"), 0, TPri_BelowNormal);
}

FA320Voice::~FA320Voice()
{
	if (Thread)
	{
		Thread->Kill(true);  // calls Stop(), then waits for Run() to return
		delete Thread;
		Thread = nullptr;
	}
	if (Wake)
	{
		FPlatformProcess::ReturnSynchEventToPool(Wake);
		Wake = nullptr;
	}
}

void FA320Voice::Speak(const FString& Text, int32 Speaker, int32 FrequencyKhz)
{
	if (VoiceCount.load() == 0 || Text.IsEmpty())
	{
		return;
	}
	Jobs.Enqueue({Text, Speaker, FrequencyKhz, Generation.load()});
	Wake->Trigger();
}

bool FA320Voice::PopClip(TArray<int16>& OutPcm, int32& OutFrequencyKhz)
{
	FClip Clip;
	do
	{
		if (!Clips.Dequeue(Clip))
		{
			return false;
		}
	} while (Clip.Generation != Generation.load());
	OutPcm = MoveTemp(Clip.Pcm);
	OutFrequencyKhz = Clip.FrequencyKhz;
	return true;
}

void FA320Voice::Stop()
{
	bStopping = true;
	bCancelSpeech = true;
	if (Wake)
	{
		Wake->Trigger();
	}
}

uint32 FA320Voice::Run()
{
	// COM and the SAPI voice live on this thread only.
	a320speech::Synthesizer Synth;
	VoiceCount = Synth.IsReady() ? Synth.VoiceCount() : 0;
	UE_LOG(LogA320, Log, TEXT("Radio voices: %d installed%s"), VoiceCount.load(),
		VoiceCount.load() > 0 ? TEXT("") : TEXT(" (ATC as text only)"));
	while (!bStopping)
	{
		FJob Job;
		if (!Jobs.Dequeue(Job))
		{
			Wake->Wait(250);
			continue;
		}
		const a320speech::Speaker Who = Job.Speaker == A320_ATC_SPEAKER_PILOT ? a320speech::Speaker::Crew
			: (Job.Speaker == A320_ATC_SPEAKER_ATIS ? a320speech::Speaker::Atis : a320speech::Speaker::Atc);
		if (Job.Generation != Generation.load())
		{
			continue;
		}
		std::wstring Text;
		Text.reserve(static_cast<size_t>(Job.Text.Len()));
		for (int32 i = 0; i < Job.Text.Len(); ++i)
		{
			Text.push_back(static_cast<wchar_t>(Job.Text[i]));
		}
		std::vector<int16_t> Pcm;
		// A flush or shutdown during a long transmission stops it.
		bCancelSpeech = bStopping.load();
		if (Synth.Speak(Text, Who, bCancelSpeech, Pcm) && Job.Generation == Generation.load())
		{
			FClip Clip;
			Clip.Generation = Job.Generation;
			Clip.Pcm.Append(Pcm.data(), static_cast<int32>(Pcm.size()));
			Clip.FrequencyKhz = Job.FrequencyKhz;
			Clips.Enqueue(MoveTemp(Clip));
		}
	}
	return 0;
}
