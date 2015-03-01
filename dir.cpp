#include <sys/types.h>
#include <dirent.h>
#include <errno.h>
#include <vector>
#include <string>
#include <iostream>
#include <stdlib.h>
#include <unistd.h>

using namespace std;

/*function... might want it in some class?*/
int getdir (string dir, vector<string> &files)
{
    DIR *dp;
    struct dirent *dirp;
    if((dp  = opendir(dir.c_str())) == NULL) {
        cout << "Error(" << errno << ") opening " << dir << endl;
        return errno;
    }

    while ((dirp = readdir(dp)) != NULL) {
        files.push_back(string(dirp->d_name));
    }
    closedir(dp);
    return 0;
}


vector<string> updateDir() {
    string dir = string(".");
    vector<string> files = vector<string>();
    getdir(dir,files);
    return files;
}

int main()
{
    while(1) {
        vector<string> files = updateDir();
        for (unsigned int i = 0;i < files.size();i++) {
            cout << files[i] << endl;
        }
        cout << "Enter an index: " << endl;

        int pick;
        cin >> pick;
     
        int dotIndex = files[pick].find_last_of('.');
        
        if(dotIndex == string::npos || files[pick].length() == dotIndex + 1) {
            chdir(files[pick].c_str());
        } else {
            string s2 = "vim " + files[pick];
            system(s2.c_str());
        }
    }

    return 0;
}