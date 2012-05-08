#include <cstring>
#include <cstdlib>

int main(int argc, char** argv){
  // Instantiate the required variables
  char command[1024];
  char path[256];

  // Build the python file path
  char* lastDir= strrchr(argv[0], '/');
  int lastIndex= (lastDir - argv[0])/sizeof(char);
  for (int i=0; i<=lastIndex; i++){
    path[i]= argv[0][i];
    path[i+1]= '\0';
  }
  strcat(path, "../src/vertis.py ");

  // Build the OS command
  strcpy(command,"python ");
  strcat(command, path);
  for (int i=1; i<argc; i++){
    if (argv[i][0]=='<') strcat(command, "\"");
    strcat(command, argv[i]);
    if (argv[i][0]=='<') strcat(command, "\"");
    strcat(command," ");
  }

  // Run OS command
  return system(command);
}
