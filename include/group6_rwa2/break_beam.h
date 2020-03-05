/*
 * breakbeam.h
 *
 *  Created on: Mar 3, 2020
 *      Author: sanket
 */
#pragma once
#ifndef BREAKBEAM_H_
#define BREAKBEAM_H_
#include <list>
#include <osrf_gear/Proximity.h>

class BreakBeam {
public:
	BreakBeam();
	~BreakBeam();
	std::list<bool> break_beam_msg;
	void breakBeamCallback(const osrf_gear::Proximity::ConstPtr& msg);
};


#endif /* BREAKBEAM_H_ */
