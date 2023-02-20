//
// Created by leova on 20/02/2023.
//

#ifndef MBED_VIA_JETBRAIN_ODOMETRIE_H
#define MBED_VIA_JETBRAIN_ODOMETRIE_H

#include <cmath>

namespace Odometrie {
    class Odometrie {
    public:
        // Attributs
        int tick_md;
        int tick_mg;
        double x;
        double y;
        double r;
        double theta;
        const int Te_odometrie = 100; // en us
        const int diametre_roue = 78; // mm
        const float delta_theta = (2 * M_PI) / (14 * 8);

        // Constructeur
        Odometrie();

        // Méthodes
        void fonction_periodique();
        void set_tick_md(int new_tick_md);
        int get_tick_md() const;
        void set_tick_mg(int new_tick_mg);
        int get_tick_mg() const;
        double get_x() const;
        double get_y() const;
        double get_r() const;
        double get_theta() const;
        void set_x(float new_x);
        void set_y(float new_y);
        void set_r(float new_r);
        void set_theta(float new_theta);
    private:
        void cart2pol();
        void pol2cart();
    };
}

#endif //MBED_VIA_JETBRAIN_ODOMETRIE_H
