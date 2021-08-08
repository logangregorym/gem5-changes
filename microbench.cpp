#include <iostream>
#include <cstdlib>

uint64_t a[100], b[100], c[100];

void sum1() {
    for (int i = 0; i < 100; i+=2) {
        c[i] = a[i] | b[i];
    }
}
void sum2() {
    for (int i = 1; i < 100; i+=2) {
        c[i] = a[i] | b[i];
    }
}

int main() { 
    for (int i = 0; i < 100; i+=2) {
        a[i] = 3;
        b[i] = 4;
    }
    for (int i = 1; i < 100; i+=2) {
        a[i] = 4;
        b[i] = 5;
    }
    for (int i = 0; i < 5000; i++) {
        sum1();
    }
    for (int i = 0; i < 2000; i++) {
        sum2();
    }
    for (int i = 0; i < 100; i++) {
        std::cout << c[i] << "\n";
    }
}
