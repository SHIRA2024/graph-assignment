
// Email: shiraba01@gmail.com
#ifndef QUEUE_HPP
#define QUEUE_HPP

class Queue {
private:
    int* data;
    int front;
    int rear;
    int size;
    int capacity;

public:
    Queue(int capacity);
    ~Queue();

    void enqueue(int value);
    int dequeue();
    bool is_empty() const;
};

#endif