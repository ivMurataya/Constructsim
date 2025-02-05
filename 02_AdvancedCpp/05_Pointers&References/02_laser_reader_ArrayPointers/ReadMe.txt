

Add to the 5.2.1 exercise program an array of pointers to arrays called last_ten_scans of size 10 as a member of the class.

Tasks:

    Store in the last_ten_scan the last 10 scans received.
    Print on the screen the range measured by the ray at the front of the robot for the last 10 scans received

Hints for Exercise 5.2:

    You will need to create a copy of the last_scan and provide the pointer to that to the 1st position of the last_ten_scans array.
    Then do the same with the second last_scan received at position number 2
    Then do the same for the third, forth, and so until ten
    Once reached the tenth position, then put the 11th scan pointer into the 1st position of the array
    Continue the same procedure forever. You will always keep the last 10 scans received into the last_ten_scans array in the form of pointers to arrays of float.
    Remember to delete the array in the destructor of the class

