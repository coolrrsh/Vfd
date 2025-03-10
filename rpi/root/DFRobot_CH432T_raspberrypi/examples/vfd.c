#include <stdio.h>

#include <stdlib.h>


int main() {

    int choice;

    

    while (1) {

        // Display menu

        printf("\nVFD Control Menu:\n");

        printf("1. Start\n");

        printf("2. Stop\n");

        printf("3. Forward\n");

        printf("4. Reverse\n");

      //  printf("5. Exit\n");

        printf("Enter command : ");

        

        // Get user input

        scanf("%d", &choice);

        

        // Execute corresponding Python command

        switch (choice) {

            case 1: system("python3 ch432t_test.py start"); break;

            case 2: system("python3 ch432t_test.py stop"); break;

            case 3: system("python3 ch432t_test.py forward"); break;

            case 4: system("python3 ch432t_test.py reverse"); break;

            case 5: printf("Exiting...\n"); return 0;

            default: printf("Invalid choice, please enter a number between 1 and 5.\n");

        }

    }

    

    return 0;

}
