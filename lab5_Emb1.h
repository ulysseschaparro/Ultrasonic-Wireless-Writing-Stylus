//Ulysses Chaparro 1001718774

#ifndef LAB5_EMB1
#define LAB5_EMB1

#define MAX_CHARS 150
#define MAX_FIELDS 8
typedef struct USER_DATA
        {
            char buffer[MAX_CHARS+1];
            uint8_t fieldCount;
            uint8_t fieldPosition[MAX_FIELDS];
            char fieldType[MAX_FIELDS];
        } USER_DATA;

void getsUart0(USER_DATA *data);
void parseFields(USER_DATA *data);
char* getFieldString(USER_DATA *data, uint8_t fieldNumber);
int32_t getFieldInteger(USER_DATA *data, uint8_t fieldNumber);
bool str_cmp(char *string1, char *string2);
bool isCommand(USER_DATA *data, const char strCommand[], uint8_t minArguments);

#endif
