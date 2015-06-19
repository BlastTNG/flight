/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef SHARED__RESULT_H
#define SHARED__RESULT_H

namespace Shared
{


template <class T>
class Result
{
  public:
    Result();
    Result& operator=(const Result& rhs);

    bool found;
    int counter;
    T value;
};

template <class T>
Result<T>::Result()
{
    found = false;
    counter = 0;
    value = 0;
}

template <class T>
Result<T>& Result<T>::operator=(const Result<T> &rhs)
{
    if (this != &rhs) {
        found = rhs.found;
        counter = rhs.counter;
        value = rhs.value;
    }
    return *this;
}


}

#endif
