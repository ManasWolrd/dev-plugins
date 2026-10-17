#pragma once
#include <array>
#include <complex>
#include <pluginshared/bpm_sync_lfo.hpp>
#include <pluginshared/preset_manager.hpp>

#include "deep_phaser.hpp"
#include "global.hpp"
#include "params.hpp"

// ---------------------------------------- juce processor ----------------------------------------
class DeepPhaserAudioProcessor final : public juce::AudioProcessor {
public:
    //==============================================================================
    DeepPhaserAudioProcessor();
    ~DeepPhaserAudioProcessor() override;

    //==============================================================================
    void prepareToPlay(double sampleRate, int samplesPerBlock) override;
    void releaseResources() override;
    void reset() override;

    bool isBusesLayoutSupported(const BusesLayout& layouts) const override;

    void processBlock(juce::AudioBuffer<float>&, juce::MidiBuffer&) override;
    using AudioProcessor::processBlock;

    //==============================================================================
    juce::AudioProcessorEditor* createEditor() override;
    bool hasEditor() const override;

    //==============================================================================
    const juce::String getName() const override;

    bool acceptsMidi() const override;
    bool producesMidi() const override;
    bool isMidiEffect() const override;
    double getTailLengthSeconds() const override;

    //==============================================================================
    int getNumPrograms() override;
    int getCurrentProgram() override;
    void setCurrentProgram(int index) override;
    const juce::String getProgramName(int index) override;
    void changeProgramName(int index, const juce::String& newName) override;

    //==============================================================================
    void getStateInformation(juce::MemoryBlock& destData) override;
    void setStateInformation(const void* data, int sizeInBytes) override;

    std::unique_ptr<juce::AudioProcessorValueTreeState> value_tree_;
    std::unique_ptr<pluginshared::PresetManager> preset_manager_;

    deepphaser::Params params_;
    deepphaser::Engine engine_;

    bool display_custom_{true};
private:
    float barber_osc_phase_sync_{};

    //==============================================================================
    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(DeepPhaserAudioProcessor)
};
