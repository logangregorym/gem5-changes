#include <iostream>
#include <cstdlib>

uint64_t a[100], b[100], c[100];

void sum() {
    for (int i = 0; i < 100; i++) {
        c[i] = a[i] | b[i];
    }
}

int main() { 
    for (int i = 0; i < 100; i++) {
        a[i] = rand() % 10000;
        b[i] = rand() % 1000;
    }
    for (int i = 0; i < 5000; i++) {
        sum();
    }
    for (int i = 0; i < 100; i++) {
        std::cout << c[i] << "\n";
    }
}
