#include <iostream>

void test_grid();
void test_astar_straight();
void test_astar_obstacle();
void test_astar_no_path();

int main() {
    test_grid();
    test_astar_straight();
    test_astar_obstacle();
    test_astar_no_path();

    std::cout << "All tests passed.\n";
    return 0;
}
