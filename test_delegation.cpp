#include <iostream>

template <typename T>
class A {
public:
    int x;
    A(int x) : x(x) { std::cout << "A(int)" << std::endl; }
    // A() : A::A(0) {} // This is what LinearSensor does roughly
    A() : A(0) {} // This is correct
};

template <typename T>
class B {
public:
    int y;
    B(int y) : y(y) { std::cout << "B(int)" << std::endl; }
    B();
};

template <typename T>
B<T>::B() : B::B(0) {} // This is how LinearSensor writes it

int main() {
    B<int> b;
    std::cout << b.y << std::endl;
    return 0;
}
