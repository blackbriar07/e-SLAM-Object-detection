#pragma once
#include <stdio.h>
#include <vector>
#include <string>

using namespace std;

double GetValueFromFile(FILE* file, string string1)
{
    //FILE* file = fopen("D:/LinearAccDeceleration_Setting.txt", "r");

    // Check if the file was opened successfully
    if (file == nullptr) {
        perror("Error opening file");
        return 1;
    }

    // Read and print each line of the file
    char buffer[256];
    vector<string> words;
    while (fgets(buffer, sizeof(buffer), file) != nullptr) {
        //printf("%s", buffer);
        string wordBeforeComma;
        string wordAfterComma;
        bool GetWord = false;
        for (int i = 0; buffer[i] != '\0'; ++i) {
            // Check if the character is a whitespace or a newline
            if (buffer[i] == ',' && !GetWord) // || isspace(buffer[i]) || buffer[i] == '\n') 
            {
                //printf("temp Word %s\n", wordBeforeComma.c_str());
                int j = wordBeforeComma.compare(string1);
                if (j == 0)
                {
                    GetWord = true;
                    //printf("Got it\n");
                }
                // Add the word to the vector if it's not empty
                //if (!wordBeforeComma.empty()) {
                //    words.push_back(wordBeforeComma);
                //    wordBeforeComma.clear();
                //}
                //break;
            }
            else if (buffer[i] == '\n')
            {
                if (!wordAfterComma.empty())
                {
                    //printf("temp Value %s\n", wordAfterComma.c_str());
                    return stod(wordAfterComma);
                }
            }
            else {
                // Append the character to the current word
                if (isspace(buffer[i]))
                    continue;
                if (!GetWord)
                    wordBeforeComma += buffer[i];
                if (GetWord)
                    wordAfterComma += buffer[i];
            }
        }
    }
    for (int i = 0; i < words.size(); i++)
    {
        printf("Word %s\n", words[i].c_str());
    }
    // Close the file
    fclose(file);
    return 0;
}

void DeleteSpaceFromString(string& str)
{
    str.erase(std::remove(str.begin(), str.end(), ' '), str.end());
}

