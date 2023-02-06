#ifndef robot_hpp
#define robot_hpp

class Robot{
    // Attributs
    // trapèze
    bool EN_trapeze;
    uint32_t T1; // en us 
    uint32_t Tc; // en us
    uint32_t T; // en us
    float Vc
    const float A_max = 2000; // ~15ms pour atteindre V_max  tick / s²
    const float V_max = 300; // ~1 m/s
    const uint32_t Te_commande = 1e4; // en us 

    // Méthodes
    void trapeze_naif(float d); // commande pour la distance
    void commande(void);
    
};


#endif