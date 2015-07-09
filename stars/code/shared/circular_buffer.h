/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef SHARED__CIRCULAR_BUFFER_H
#define SHARED__CIRCULAR_BUFFER_H

//  A circular buffer should be used when there are frequent reads
//  In between calls to update(), only num_free_slots calls to share() will be effective

#include "thread_names.h"

namespace Parameters
{
    class Manager;
}

namespace Shared
{
    template <class T> class CircularBufferParent;
    template <class T> class CircularBuffer;
    template <class T> class CircularBufferPass;
}

template <class T>
class Shared::CircularBufferParent
{
  public:
    CircularBufferParent(ThreadNames::Name writer_thread_, ThreadNames::Name reader_thread_, int num_free_slots=2);
    ~CircularBufferParent();
    void init(Parameters::Manager& params);
    void share();
    void retry_share(ThreadNames::Name writer_thread_);

    T* w;
    T* r;
    int size;
    bool last_share_failed;
    T* states;
    int write_index;
    int read_index;
    ThreadNames::Name writer_thread;
    ThreadNames::Name reader_thread;

protected:
    bool update();
};

template <class T>
Shared::CircularBufferParent<T>::CircularBufferParent(ThreadNames::Name writer_thread_, ThreadNames::Name reader_thread_, int num_free_slots)
{
    if (num_free_slots < 1) {
        num_free_slots = 1;
    }
    size = num_free_slots + 2;
    last_share_failed = false;
    states = new T[size];

    w = &states[1];
    write_index = 1;
    r = &states[0];
    read_index = 0;

    writer_thread = writer_thread_;
    reader_thread = reader_thread_;
}

template <class T>
Shared::CircularBufferParent<T>::~CircularBufferParent()
{
    delete [] states;
}

template <class T>
void Shared::CircularBufferParent<T>::init(Parameters::Manager& params)
{
    for (int i=0; i<size; i++) {
        states[i].init(params);
    }
}

// this function called by write-accessing thread
template <class T>
void Shared::CircularBufferParent<T>::share()
{
    int next_write_index = write_index + 1;
    if (next_write_index >= size) {
        next_write_index = 0;
    }
    if (next_write_index != read_index) {
        states[next_write_index] = states[write_index];
        w = &states[next_write_index];
        write_index = next_write_index;
        last_share_failed = false;
    } else {
        last_share_failed = true;
    }
}

template <class T>
void Shared::CircularBufferParent<T>::retry_share(ThreadNames::Name writer_thread_)
{
    if (writer_thread == writer_thread_) {
        if (last_share_failed) {
            share();
        }
    }
}


// this function called by read-accessing thread
template <class T>
bool Shared::CircularBufferParent<T>::update()
{
    bool updated = false;
    int next_read_index = write_index - 1;
    if (next_read_index < 0) {
        next_read_index = size - 1;
    }
    if (next_read_index != read_index) {
        r = &states[next_read_index];
        read_index = next_read_index;
        updated = true;
    }
    return updated;
}





template <class T>
class Shared::CircularBuffer: public Shared::CircularBufferParent<T>
{
  public:
    CircularBuffer(ThreadNames::Name writer_thread_ = ThreadNames::nobody,
                   ThreadNames::Name reader_thread_ = ThreadNames::nobody);
    bool update(ThreadNames::Name thread_name);
};

template <class T>
Shared::CircularBuffer<T>::CircularBuffer(ThreadNames::Name writer_thread_,
                                                  ThreadNames::Name reader_thread_):
    CircularBufferParent<T>(writer_thread_, reader_thread_)
{
}

template <class T>
bool Shared::CircularBuffer<T>::update(ThreadNames::Name thread_name)
{
    if (this->reader_thread == thread_name) {
        return CircularBufferParent<T>::update();
    }
    return false;
}





template <class T>
class Shared::CircularBufferPass: public Shared::CircularBufferParent<T>
{
  public:
    CircularBufferPass(ThreadNames::Name writer_thread_ = ThreadNames::nobody,
                       ThreadNames::Name reader_thread_ = ThreadNames::nobody);
    void update_and_pass_to(Shared::CircularBufferParent<T>& writer, ThreadNames::Name thread_name);
};

template <class T>
Shared::CircularBufferPass<T>::CircularBufferPass(ThreadNames::Name writer_thread_,
                                                  ThreadNames::Name reader_thread_):
    CircularBufferParent<T>(writer_thread_, reader_thread_)
{
}

// this function called by read-accessing thread of this, and the write-accessing thread of writer
template <class T>
void Shared::CircularBufferPass<T>::update_and_pass_to(Shared::CircularBufferParent<T>& writer, ThreadNames::Name thread_name)
{
    if (writer.writer_thread == thread_name && this->reader_thread == thread_name) {
        if (CircularBufferParent<T>::update()) {
            *(writer.w) = *(this->r);
            writer.share();
        } else {
            writer.retry_share(thread_name);
        }
    }
}


#endif

