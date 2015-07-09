/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef SHARED__REQUEST_H
#define SHARED__REQUEST_H

namespace Shared
{


template <class T>
class Request
{
  public:
    Request();
    Request& operator=(const Request& rhs);

    int counter;
    T value;
};

template <class T>
Request<T>::Request()
{
    counter = 0;
    value = 0;
}

template <class T>
Request<T>& Request<T>::operator=(const Request<T> &rhs)
{
    if (this != &rhs) {
        counter = rhs.counter;
        value = rhs.value;
    }
    return *this;
}


}

#endif
