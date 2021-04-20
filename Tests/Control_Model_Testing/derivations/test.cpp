#include <stdio.h>


int func_two(int *error){
    printf("func 1 error value is: %d\n", *error);
    return 0;
}


int func_one(int *error){
    printf("func 1 error value is: %d\n", *error);
    *error = 2;
    func_two(error);
    return 0;
}


int main(void){
    int error = 1;
    func_one(&error);

    return 0;
}
