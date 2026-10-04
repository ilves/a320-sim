#pragma once

#include <atomic>
#include <cstdint>
#include <string>
#include <vector>

// Windows text-to-speech (SAPI 5) rendered into memory as 16-bit mono PCM, for the radio. One
// instance per thread: it initialises COM on the thread that creates it.
namespace a320speech
{
	constexpr int kSampleRate = 22050;

	enum class Speaker
	{
		Atc,
		Crew,
		Atis
	};

	class Synthesizer
	{
	public:
		Synthesizer();
		~Synthesizer();
		Synthesizer(const Synthesizer&) = delete;
		Synthesizer& operator=(const Synthesizer&) = delete;

		bool IsReady() const;
		int VoiceCount() const;
		std::wstring VoiceName(int Index) const;
		// The speech of one radio transmission; false if nothing could be spoken or Cancel was set
		// (polled while speaking, so shutting down never waits for a long ATIS).
		bool Speak(const std::wstring& Text, Speaker Who, const std::atomic<bool>& Cancel, std::vector<int16_t>& OutPcm);

	private:
		struct FImpl;
		FImpl* Impl = nullptr;
	};
}
