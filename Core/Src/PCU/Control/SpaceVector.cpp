#include "PCU/Control/SpaceVector.hpp"
#include "PCU/PCU.hpp"

float SpaceVector::Imodulation = 0.0f;
float SpaceVector::Modulation_frequency = 0.0f;

static constexpr float TWO_PI = 2.0f * M_PI;
static constexpr float phase_shift = 2 * M_PI / 3;
static constexpr float INV_TWO_PI = 1.0f / TWO_PI;
static constexpr float PI_HALF = M_PI / 2.0f;



void SpaceVector::set_target_voltage(float V_ref) {
    if (V_ref < 0) V_ref = 0;
    Imodulation = V_ref * 2.0 / VMAX;
    if (Imodulation >= IMAX) Imodulation = IMAX;
    PCU::control_data.imod = Imodulation;
}

void SpaceVector::set_frequency_Modulation(float freq) {
    Modulation_frequency = freq;
    PCU::control_data.modulation_frequency = Modulation_frequency;
}

void SpaceVector::calculate_duties() {
    constexpr float period_time = static_cast<float>(Period) / 1'000'000.0f;    
    
    float delta_phase = TWO_PI * Modulation_frequency * period_time;
    phase_rad += delta_phase;

    int rotations = static_cast<int>(phase_rad * INV_TWO_PI);
    phase_rad = phase_rad - (rotations * TWO_PI);
    if (phase_rad < 0.0f) {
        phase_rad += TWO_PI;
    }

#if MODE_CALCULATE_SIN == 0
    float sin_u = Imodulation * sin(phase_rad);
    float sin_v = Imodulation * sin(phase_rad + phase_shift);
    float sin_w = Imodulation * sin(phase_rad - phase_shift);
#endif
#if MODE_CALCULATE_SIN == 1
    float sin_u = Imodulation* calculate_sin_phase(phase::U);
    float sin_v = Imodulation* calculate_sin_phase(phase::V);
    float sin_w = Imodulation* calculate_sin_phase(phase::W);
#endif

#if ARMONIC_INJECTION == 1
    float offset =  (std::max({sin_u, sin_v, sin_w}) + std::min({sin_u, sin_v, sin_w})) / 2;
    sin_u -= offset;
    sin_v -= offset;
    sin_w -= offset;
#endif

    if ( Comms::Reverse_direction == false){ 
        PWMActuators::set_duty_u((sin_u / 2.0 + 0.5) * 100.0);
        PWMActuators::set_duty_v((sin_v / 2.0 + 0.5) * 100.0);
    } else {
        PWMActuators::set_duty_u((sin_v / 2.0 + 0.5) * 100.0);
        PWMActuators::set_duty_v((sin_u / 2.0 + 0.5) * 100.0);
    }
    PWMActuators::set_duty_w((sin_w / 2.0 + 0.5) * 100.0);
  
    time += period_time;

    if(Modulation_frequency > 0.0f && time >= (2.0))
    {
        time -= 2.0f;
    }

    PCU::control_data.time = time;
}

float SpaceVector::get_modulation_frequency() {
    return Modulation_frequency;
}

float SpaceVector::get_actual_time() { return time; }
void SpaceVector::set_VMAX(float Vmax) { VMAX = Vmax; }

#if MODE_CALCULATE_SIN == 1
float SpaceVector::calculate_sin_look_up_table(float angle) {
    int rotations = static_cast<int>(angle * INV_TWO_PI);
    angle = angle - (rotations * TWO_PI);

    if (angle < 0.0f) {
        angle += TWO_PI;
    }

    float sign = 1.0f;
    if (angle > M_PI) {
        angle = TWO_PI - angle;
        sign = -1.0f;
    }

    if (angle > (PI_HALF)) {
        angle = M_PI - angle;
    }

    constexpr float SCALE = static_cast<float>(NUMBER_POINTS) / (M_PI / 2.0f);
    
    float val = angle * SCALE;
    int idx = static_cast<int>(val);
    float interpolation = val - static_cast<float>(idx);

    if (idx >= NUMBER_POINTS - 1) {
        return sign * look_up_table_sin[NUMBER_POINTS - 1];
    }

    float result = look_up_table_sin[idx] * (1.0f - interpolation) + 
                   look_up_table_sin[idx + 1] * interpolation;

    return sign * result;
}
float SpaceVector::calculate_sin_phase(phase p) {
    float angle = 0.0f;
    if (p == phase::U) {
        angle = phase_rad;
        return calculate_sin_look_up_table(angle);
    }
    else if (p == phase::V) {
        angle = phase_rad + phase_shift;
        return calculate_sin_look_up_table(angle);
    }
    else if (p == phase::W) {
        angle = phase_rad - phase_shift;
        return calculate_sin_look_up_table(angle);
    }
    return calculate_sin_look_up_table(angle);
}
#endif