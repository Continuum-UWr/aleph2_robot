#include <string>
#include <iostream>

int main(int argc, char **argv)
{
    uint count_min = 0;
    uint count_max = 1000000;
    uint limit;
    std::cin>>limit;
    while (count_max - count_min > 0)
    {
        uint mid = (count_max + count_min) / 2;
        if (mid<limit)
            count_min = mid + 1;
        else
            count_max = mid;
    }
    std::cout<<count_max<<" "<<count_min;
}