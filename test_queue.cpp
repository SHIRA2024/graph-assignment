// Email: shiraba01@gmail.com

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "Queue.hpp"



TEST_CASE("Queue initialization") {
    Queue q(3);
    CHECK(q.is_empty() == true);
}

TEST_CASE("Enqueue and dequeue operations") {
    Queue q(3);
    q.enqueue(10);
    q.enqueue(20);
    CHECK(q.is_empty() == false);
    CHECK(q.dequeue() == 10);
    CHECK(q.dequeue() == 20);
    CHECK(q.is_empty() == true);
}

TEST_CASE("Dequeue from empty queue throws error") {
    Queue q(2);
    CHECK_THROWS_AS(q.dequeue(), std::underflow_error);
}

TEST_CASE("Enqueue beyond capacity throws error") {
    Queue q(2);
    q.enqueue(1);
    q.enqueue(2);
    CHECK_THROWS_AS(q.enqueue(3), std::overflow_error);
}

TEST_CASE("Circular behavior of queue") {
    Queue q(2);
    q.enqueue(1);
    q.enqueue(2);
    CHECK(q.dequeue() == 1);
    q.enqueue(3);
    CHECK(q.dequeue() == 2);
    CHECK(q.dequeue() == 3);
    CHECK(q.is_empty() == true);
}
