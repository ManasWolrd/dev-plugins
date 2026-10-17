#include "PluginProcessor.h"
#include "PluginEditor.h"

#include <algorithm>
#include <complex>

//==============================================================================
DeepPhaserAudioProcessor::DeepPhaserAudioProcessor()
    : AudioProcessor(BusesProperties()
#if !JucePlugin_IsMidiEffect
#if !JucePlugin_IsSynth
                         .withInput("Input", juce::AudioChannelSet::stereo(), true)
#endif
                         .withOutput("Output", juce::AudioChannelSet::stereo(), true)
#endif
      ) {
    juce::AudioProcessorValueTreeState::ParameterLayout layout;
    params_.BuildLayout(layout);

    value_tree_ = std::make_unique<juce::AudioProcessorValueTreeState>(*this, nullptr, "PARAMETERS", std::move(layout));
    params_.BeginListening();
    preset_manager_ = std::make_unique<pluginshared::PresetManager>(*value_tree_, *this);

    // 默认零极点布局
    deepphaser::Params::InitDefaultPZ(params_.poles_, params_.zeros_);
}

DeepPhaserAudioProcessor::~DeepPhaserAudioProcessor() {
    params_.EndListening();
    value_tree_ = nullptr;
}

//==============================================================================
const juce::String DeepPhaserAudioProcessor::getName() const {
    return JucePlugin_Name;
}

bool DeepPhaserAudioProcessor::acceptsMidi() const {
#if JucePlugin_WantsMidiInput
    return true;
#else
    return false;
#endif
}

bool DeepPhaserAudioProcessor::producesMidi() const {
#if JucePlugin_ProducesMidiOutput
    return true;
#else
    return false;
#endif
}

bool DeepPhaserAudioProcessor::isMidiEffect() const {
#if JucePlugin_IsMidiEffect
    return true;
#else
    return false;
#endif
}

double DeepPhaserAudioProcessor::getTailLengthSeconds() const {
    return 0.0;
}

int DeepPhaserAudioProcessor::getNumPrograms() {
    return 1;
}

int DeepPhaserAudioProcessor::getCurrentProgram() {
    return 0;
}

void DeepPhaserAudioProcessor::setCurrentProgram(int index) {
    juce::ignoreUnused(index);
}

const juce::String DeepPhaserAudioProcessor::getProgramName(int index) {
    juce::ignoreUnused(index);
    return {};
}

void DeepPhaserAudioProcessor::changeProgramName(int index, const juce::String& newName) {
    juce::ignoreUnused(index, newName);
}

//==============================================================================
void DeepPhaserAudioProcessor::prepareToPlay(double sampleRate, int samplesPerBlock) {
    std::ignore = samplesPerBlock;
    engine_.Init(static_cast<float>(sampleRate));
    engine_.Reset();

    // 首次进入时重建系数
    params_.should_update_iir_.store(true, std::memory_order_release);
}

void DeepPhaserAudioProcessor::releaseResources() {}

void DeepPhaserAudioProcessor::reset() {
    engine_.Reset();
}

bool DeepPhaserAudioProcessor::isBusesLayoutSupported(const BusesLayout& layouts) const {
#if JucePlugin_IsMidiEffect
    juce::ignoreUnused(layouts);
    return true;
#else
    if (layouts.getMainOutputChannelSet() != juce::AudioChannelSet::stereo())
        return false;

#if !JucePlugin_IsSynth
    if (layouts.getMainOutputChannelSet() != layouts.getMainInputChannelSet())
        return false;
#endif

    return true;
#endif
}

void DeepPhaserAudioProcessor::processBlock(juce::AudioBuffer<float>& buffer, juce::MidiBuffer& midiMessages) {
    std::ignore = midiMessages;
    juce::ScopedNoDenormals noDenormals;

    engine_.Update(params_);

    // bpm 同步 LFO（仅 barberpole 保留）
    auto barber_info = params_.barber_lfo.SyncBpm2(getPlayHead());
    if (barber_info.sync_lfo) {
        barber_osc_phase_sync_ = barber_info.lfo_phase * std::numbers::pi_v<float> * 2;
    }

    // 快照参数（音频线程所需）
    deepphaser::Engine::Snapshot s;
    s.fs = static_cast<float>(getSampleRate());
    s.drywet = params_.drywet.Get();
    s.barber_phase = params_.barber_phase.Get();
    s.barber_enable = params_.barber_enable.Get();
    s.barber_stereo_phase = params_.barber_stereo.Get() * std::numbers::pi_v<float> / 2;
    s.state = params_.state.Get();
    s.blend = params_.blend.Get();
    s.barber_speed = barber_info.lfo_freq;

    int num_samples = buffer.getNumSamples();
    auto* left_ptr = buffer.getWritePointer(0);
    auto* right_ptr = buffer.getWritePointer(1);

    engine_.ProcessBlock(left_ptr, right_ptr, num_samples, s);
}

//==============================================================================
bool DeepPhaserAudioProcessor::hasEditor() const {
    return true;
}

juce::AudioProcessorEditor* DeepPhaserAudioProcessor::createEditor() {
    return new DeepPhaserAudioProcessorEditor(*this);
}

//==============================================================================
void DeepPhaserAudioProcessor::getStateInformation(juce::MemoryBlock& destData) {
    suspendProcessing(true);

    // 锁内仅复制数组
    std::array<global::PolarPoint, global::kMaxSections> poles_snapshot{};
    std::array<global::PolarPoint, global::kMaxSections> zeros_snapshot{};
    {
        const juce::SpinLock::ScopedLockType lock(params_.poles_zeros_lock_);
        std::copy_n(params_.poles_.begin(), global::kMaxSections, poles_snapshot.begin());
        std::copy_n(params_.zeros_.begin(), global::kMaxSections, zeros_snapshot.begin());
    }

    // 控制点以 (归一化极角 X, 模长 R) 序列化
    juce::ValueTree pz{"POLE_ZERO"};
    for (size_t i = 0; i < global::kMaxSections; ++i) {
        pz.appendChild(
            {
                "ITEM",
                {
                  {"P_X", poles_snapshot[i].x},
                  {"P_R", poles_snapshot[i].r},
                  {"Z_X", zeros_snapshot[i].x},
                  {"Z_R", zeros_snapshot[i].r},
                  }
        },
            nullptr);
    }

    juce::ValueTree plugin_state{"PLUGIN_STATE"};
    plugin_state.appendChild(value_tree_->copyState(), nullptr);
    plugin_state.appendChild(pz, nullptr);

    if (auto xml = plugin_state.createXml(); xml != nullptr) {
        copyXmlToBinary(*xml, destData);
    }

    suspendProcessing(false);
}

void DeepPhaserAudioProcessor::setStateInformation(const void* data, int sizeInBytes) {
    suspendProcessing(true);

    auto xml = *getXmlFromBinary(data, sizeInBytes);
    auto plugin_state = juce::ValueTree::fromXml(xml);

    if (plugin_state.isValid()) {
        auto parameters = plugin_state.getChildWithName("PARAMETERS");
        if (parameters.isValid()) {
            value_tree_->replaceState(parameters);
        }
        else {
            // 无参数状态时仍保证默认布局已初始化
        }

        auto pz = plugin_state.getChildWithName("POLE_ZERO");
        if (pz.isValid()) {
            const juce::SpinLock::ScopedLockType lock(params_.poles_zeros_lock_);
            // 默认布局作为未填充槽位的回退
            deepphaser::Params::InitDefaultPZ(params_.poles_, params_.zeros_);
            size_t i = 0;
            for (auto item : pz) {
                if (i >= global::kMaxSections)
                    break;
                params_.poles_[i] = {static_cast<float>(item.getProperty("P_X", 0.5)),
                                     static_cast<float>(item.getProperty("P_R", 0.8))};
                params_.zeros_[i] = {static_cast<float>(item.getProperty("Z_X", 0.5)),
                                     static_cast<float>(item.getProperty("Z_R", 1.0))};
                ++i;
            }
            params_.should_update_iir_.store(true, std::memory_order_release);
        }
    }

    reset();
    suspendProcessing(false);
}

//==============================================================================
juce::AudioProcessor* JUCE_CALLTYPE createPluginFilter() {
    return new DeepPhaserAudioProcessor();
}
