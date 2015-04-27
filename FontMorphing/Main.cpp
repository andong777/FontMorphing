#include "FontMorphing.h"

#ifdef DEMO_MODE
bool toScreen = true;
#else
bool toScreen = false;
#endif

string sourceCharDir = "E:\\TestData\\PKU\\HT";
string targetCharDir = "E:\\TestData\\PKU\\LS";
string charListPath = "E:\\TestData\\PKU\\HT\\list.txt";
string outputCharDir = "E:\\output";

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
	return 0;
}