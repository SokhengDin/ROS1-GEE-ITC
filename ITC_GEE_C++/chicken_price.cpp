#include <iostream>
#include <string>
using namespace std;

int main()
{
    string chickenGender;
    string quality;
    string displayCond;
    double chickenWeight;
    double qualityFactor;
    double numberOfChicken;
    double chickenValue;
    double chickenPriceUnit;
    double chickenSellPrice;
    cout << "Enter the gender of the chicken: ";
    cin >> chickenGender;
    cout << "Enter the weight of the chicken: ";
    cin >> chickenWeight;
    cout << "Enter the quality of the chicken: ";
    cin >> quality;
    cout << "Enter the number of the chicken: ";
    cin >> numberOfChicken;

    if (chickenGender == "M")
    {
        chickenPriceUnit = 15000;
        if (quality == "Good")
        {
            qualityFactor = 1;
        }
        else if (quality == "Bad")
        {
            qualityFactor = 0.7;
        }
        chickenSellPrice = qualityFactor * chickenPriceUnit * chickenWeight * numberOfChicken;
    }
    else if (chickenGender == "F")
    {
        chickenPriceUnit = 20000;
        if (quality == "Good")
        {
            qualityFactor = 1;
        }
        else if (quality == "Bad")
        {
            qualityFactor = 0.7;
        }
        chickenSellPrice = qualityFactor*chickenPriceUnit*chickenWeight*numberOfChicken;
    }
    cout << "Do you want to display money as riel or dollar ??" << endl;
    cout << "Enter: " ;
    cin >> displayCond;
    if (displayCond == "real")
    {
        cout << "The total money of sell chickens: " << chickenSellPrice << " riel " << endl;
    }
    else if(displayCond == "dollar")
    {
        cout << "The total money of selling chickens: " << (chickenSellPrice/4000) << " $ " << endl;
    }
    return 0;
}