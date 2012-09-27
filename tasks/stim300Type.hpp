#ifndef STIM300_TYPE_HPP
#define STIM300_TYPE_HPP

#include <base/time.h>
#include <base/temperature.h>
#include <vector>

namespace stim300 {
    
    struct Temperature
    {
	base::Time time;
	std::vector<base::Temperature> temp;
	
	void resize(int size)
        {
                temp.resize(size);
        }
    };
}

#endif