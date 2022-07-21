#include <QString>
#include <QtTest>
#include "ctdataset.h"
#include <algorithm>

class MyLibUnitTest : public QObject
{
    Q_OBJECT

public:
    MyLibUnitTest();

private Q_SLOTS:
   void windowingTest();

};

MyLibUnitTest::MyLibUnitTest()
{
}

/**
 Test cases for CTDataset::windowing(...)
 HIER OBEN kurze Beschreibung des Testfalls in eigenen Worten einfügen, z.B. die erlaubten Grenzen einmal nennen
 */
void MyLibUnitTest::windowingTest()
{
    // VALID case 1: testing clean zero for bottom HU boundary
    int returnedVal = 1;
    int returnCode  = 0;
    returnCode = CTDataset::windowing(  -34,   -34,   100, returnedVal);
    QVERIFY2(returnCode  == 0, "returns an error although input is valid");
    QVERIFY2(returnedVal == 0, "windowing function lower bound");

    // VALID case 2: testing center of windowed domain
    returnCode  =  0;
    returnedVal = -1;
    returnCode = CTDataset::windowing(   50,     0,   100, returnedVal);
    QVERIFY2(returnCode  == 0, "returns an error although input is valid");
    QVERIFY2(returnedVal == 128, qPrintable( QString("windowing function medium value, was %1 instead of 128").arg(returnedVal) ) );

    // VALID case 3: testing top of windowed domain
    returnCode  =  0;
    returnedVal = -1;
    returnCode = CTDataset::windowing(   50,     0,   50, returnedVal);
    QVERIFY2(returnCode  == 0, "returns an error although input is valid");
    QVERIFY2(returnedVal == 255, qPrintable( QString("windowing function maximum value, was %1 instead of 255").arg(returnedVal) ) );

    // INVALID case 1: HU input too low
    returnedVal = 0;
    returnCode  = 0;
    returnCode = CTDataset::windowing(-4100, -1000,  2000, returnedVal);
    QVERIFY2(returnCode == 1, "No error code returned although input HU value was <-1024");

    // INVALID case 2: HU input too high
    returnedVal = 0;
    returnCode  = 0;
    returnCode = CTDataset::windowing( 3100,  -100,  2000, returnedVal);
    QVERIFY2(returnCode == 1, "No error code returned although input HU value was >3071");

    // INVALID case 3: HU start too low
    returnedVal = 0;
    returnCode  = 0;
    returnCode = CTDataset::windowing(  100, -1500,  1000, returnedVal);
    QVERIFY2(returnCode == 2, "Incorrect error code returned although windowing start value was <-1024");

    // INVALID case 4: HU start too high
    returnedVal = 0;
    returnCode  = 0;
    returnCode = CTDataset::windowing(  100,  3500,  1000, returnedVal);
    QVERIFY2(returnCode == 2, "Incorrect error code returned although windowing start value was >3071");

    // INVALID case 5: window width too low
    returnedVal = 0;
    returnCode  = 0;
    returnCode = CTDataset::windowing(  100,   100,  -100, returnedVal);
    QVERIFY2(returnCode == 3, "Incorrect error code returned although window width was <0");

    // INVALID case 6: window width too high
    returnedVal = 0;
    returnCode  = 0;
    returnCode = CTDataset::windowing(  100,   100,  4100, returnedVal);
    QVERIFY2(returnCode == 3, "Incorrect error code returned although window width was >4095");

}


QTEST_APPLESS_MAIN(MyLibUnitTest)

#include "tst_mylibunittest.moc"
