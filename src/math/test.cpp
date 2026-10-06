#include <iostream>
#include "Vec2.h"

void test_Vec2(){
    Vec2 a(10, 20);
    Vec2 b(10, 20);

    std::cout << b.x << " " << a.y << " " << (a+b).x;
}

int main(){
    Vec2 v(3, 4);

std::cout << v.length();
    
}