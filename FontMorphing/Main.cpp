#include "FontMorphing.h"
#include <ctime>

#ifdef DEMO_MODE
bool toScreen = true;
#else
bool toScreen = false;
#endif

string sourceCharDir = "TestData\\PKU\\HT";
string targetCharDir = "TestData\\PKU\\LS";
string charListPath = "TestData\\PKU\\testlist.txt";
string outputCharDir = "E:\\output";
time_t tstart, tend;
int main(int argc, char** argv)
{
	if (argc >= 5){
		sourceCharDir = argv[1];
		targetCharDir = argv[2];
		charListPath = argv[3];
		outputCharDir = argv[4];
		cout << "use user setting: " << endl;
	}
	else{
		cout << "use default demo data: " << endl;
	}
	cout << "source character directory: " << sourceCharDir << endl;
	cout << "target character directory: " << targetCharDir << endl;
	cout << "character list path: " << charListPath << endl;
	cout << "output character directory: " << outputCharDir << endl;
	vector<string> charList;
	ifstream f(charListPath);
	if (f){
		string thisCharName;
		while (f >> thisCharName){
			charList.push_back(thisCharName);
		}
		f.close();
	}
	else{
		cout << "character list not found! Exit" << endl;
		return -1;
	}
	tstart = time(0);
#ifdef PARALLEL_MODE
#pragma omp parallel for /*num_threads(4)*/
#endif
	for (int i = 0; i < charList.size();i++){
		string charName = charList[i];
		cout << endl << "*** processing character " << charName << "... ***" << endl << endl;
		FontMorphing *morphing = new FontMorphing(charName);
		bool done = morphing->work(toScreen);
		if (done){
			cout << "*** character " << charName << "done ***" << endl;
		}
		else{
			cout << "!!! character " << charName << "failed !!!" << endl;
		}
		delete morphing;
	}
	tend = time(0);
	ofstream of("time.txt");
	int seconds = difftime(tend, tstart);
	int hours = seconds / 3600;
	seconds %= 3600;
	int minutes = seconds / 60;
	seconds %= 60;
	of << "" << hours + " h " << minutes << " m " << seconds << " s." << endl;
	of.close();
	return 0;
}