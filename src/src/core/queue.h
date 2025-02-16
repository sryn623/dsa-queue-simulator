// src/core/queue.h
#ifndef QUEUE_H
#define QUEUE_H

template <typename T>
class Queue {
private:
    struct Node {
        T data;
        Node* next;
        Node(const T& value) : data(value), next(nullptr) {}
    };

    Node* front;
    Node* rear;
    int size;

public:
    Queue() : front(nullptr), rear(nullptr), size(0) {}
    ~Queue() {
        while (!empty()) {
            dequeue();
        }
    }

    void enqueue(const T& value) {
        Node* newNode = new Node(value);
        if (empty()) {
            front = rear = newNode;
        } else {
            rear->next = newNode;
            rear = newNode;
        }
        size++;
    }

    void dequeue() {
        if (empty()) return;

        Node* temp = front;
        front = front->next;
        delete temp;
        size--;

        if (empty()) {
            rear = nullptr;
        }
    }

    T& peek() {
        if (empty()) {
            throw std::runtime_error("Queue is empty");
        }
        return front->data;
    }

    bool empty() const { return front == nullptr; }
    int getSize() const { return size; }
};

#endif // QUEUE_H