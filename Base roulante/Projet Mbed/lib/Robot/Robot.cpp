#include "Robot.hpp"
#include "stdint.h"
#include <cstdint>


void Robot::trapeze_naif(float d){
    if (EN_trapeze == true){
        float Tc_float = 0.; 
        float T1_float = 0.; 
        float T_float = 0.; 
        bool validation = false;
        float v_pourcent = 1; // réduit virtuellement V_max en cas de courte distance
        float d_pourcent = 0.6; // pseudo constant
        float acceleration = A_max;
        float vitesse = V_max;
        // Oubli que tu n'as aucune chance et fonce
        Tc_float = (d * d_pourcent) / (vitesse * v_pourcent);
        T1_float = acceleration / vitesse;
        T_float = 2*T1_float + Tc_float;
        validation = (acceleration <= A_max) && (vitesse <= V_max) && (Tc_float < 0);

        // Si ça ne passe pas :
        while(validation){
            // réduction de la vitesse maximale et de la distance à faire à vitesse constante
            if (d_pourcent <= 0.6 && d_pourcent >= 0.3){
                d_pourcent -= 0.1;
            }
            if (d_pourcent == 0.3 && v_pourcent >= 0.2){
                v_pourcent -= 0.1;
            }
            // On recalcule
            Tc = (d * d_pourcent) / (vitesse * v_pourcent);
            T1 = acceleration / (vitesse * v_pourcent);
            T = 2*T1 + Tc;
            
            validation = (acceleration <= A_max) && (vitesse <= V_max) && (Tc > 0);
        }
        T1 = (uint32_t) (T1_float/1e6);
        Tc = (uint32_t) (Tc_float/1e6);
        T = (uint32_t) (T_float/1e6);
        Vc = vitesse * v_pourcent;
    }
}

void Robot::commande(){
    
}
