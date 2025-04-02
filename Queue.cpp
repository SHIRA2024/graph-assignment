// Email: shiraba01@gmail.com

#include "Queue.hpp"
#include <stdexcept>


/**
 * Constructor - initializes a queue with a given capacity.
 * @param cap The maximum number of elements the queue can hold.
 */
Queue::Queue(int cap) {
    capacity = cap;
    data = new int[capacity];
    front = 0;
    rear = -1;
    size = 0;
}

/**
 * Destructor - releases the memory allocated for the queue.
 */
Queue::~Queue() {
    delete[] data;
}


/**
 * Adds an element to the rear of the queue.
 * @param value The value to be added to the queue.
 * @throws std::overflow_error if the queue is full.
 */
void Queue::enqueue(int value) {
    if (size == capacity) {
        throw std::overflow_error("Queue is full");
    }
    rear = (rear + 1) % capacity;
    data[rear] = value;
    size++;
}

/**
 * Removes and returns the front element of the queue.
 * @return The element at the front of the queue.
 * @throws std::underflow_error if the queue is empty.
 */

int Queue::dequeue() {
    if (is_empty()) {
        throw std::underflow_error("Queue is empty");
    }
    int value = data[front];
    front = (front + 1) % capacity;
    size--;
    return value;
}


/**
 * Checks if the queue is empty.
 * @return true if the queue has no elements, false otherwise.
 */

bool Queue::is_empty() const {
    if (size == 0) {
        return true;
    } else {
        return false;
    }
}

