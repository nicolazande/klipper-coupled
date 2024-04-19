// Cartesian kinematics stepper pulse time generation
//
// Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
#include <stddef.h> // offsetof
#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // __visible
#include "itersolve.h" // struct stepper_kinematics
#include "pyhelper.h" // errorf
#include "trapq.h" // move_get_coord

struct coupled_stepper
{
    struct stepper_kinematics sk;
    double ka, kb;
};

static double
calc_position(struct stepper_kinematics *sk, struct move *m, double move_time)
{
    struct coupled_stepper *cs = container_of(sk, struct coupled_stepper, sk);
    struct coord c = move_get_coord(m, move_time);
    return -cs->ka*c.x + cs->kb*c.y;
}

struct stepper_kinematics * __visible
coupled_stepper_alloc(double ka, double kb)
{
    struct coupled_stepper *cs = malloc(sizeof(*cs));
    memset(cs, 0, sizeof(*cs));
    cs->ka = ka;
    cs->kb = kb;
    cs->sk.calc_position_cb = calc_position;
    cs->sk.active_flags = AF_X | AF_Y;
    return &cs->sk;
}
