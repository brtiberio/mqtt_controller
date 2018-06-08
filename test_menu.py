import os
import signal

# shortcut for clear console
# def clear():
#     os.system('cls' if os.name=='nt' else 'clear')


class Menu(object):

    """Base class for the menu"""

    def __init__(self, name, buttons):
        # Initialize values
        self.name = name
        self.buttons = buttons
        self.exitFlag = False

    def clear(self):
        os.system('cls' if os.name=='nt' else 'clear')

    def display(self):

        """Displaying the menu alongside the navigation elements"""

        self.clear()
        # Display menu name
        print(self.name)

        # Display menu buttons
        for button in self.buttons:
            print("   ", button.nav, button.name)

        # Wait for user input
        return self.userInput()

    def userInput(self):

        """Method to check and act upon user's input"""

        # This holds the amount of errors for the
        # navigation element to input comparison.
        errSel = 0
        inputSel = input("Enter selection> ")

        for button in self.buttons:
            # If input equals to button's navigation element
            if inputSel == str(button.nav):
                # Do the button's function
                return button.nav
            # If input != navigation element
            else:
                # Increase "error on selection" by one, for
                # counting the errors and checking their
                # amount against the total number of
                # buttons. If greater to or equal that means
                # no elements were selected.
                # In that case show error and try again
                errSel += 1

        # No usable input, try again
        if errSel >= len(self.buttons):
            print("Error on selection; try again.")
            return None

class Button(object):

    """Base class for menu buttons"""

    def __init__(self, name, nav):
        # Initialize values
        self.name = name
        # Navigation element; number which user has to enter to do button action
        self.nav = nav

def main():
    def showName():
        print ("My name is...")

    def showVersion():
        print ("My version is 0.1")

    def showAbout():
        print("I am a demo app for testing menus")

    def stop():
        nonlocal exitFlag
        exitFlag = True
        print("Exiting now...")

    def runCommand():
        userInput = input("Enter command> ")
        try:
            exec(userInput)
        except Exception as e:
            print('Got execption {0}... exiting now'.format(e))

    mainMenuButtonName = Button("Show name", 1)
    mainMenuButtonVersion = Button("Show version", 2)
    mainMenuButtonAbout = Button("Show about", 3)
    mainMenuButtonEval = Button("Run Command", 4)
    mainMenuButtonQuit = Button("Quit", 0)
    mainMenuButtons = [mainMenuButtonName, mainMenuButtonVersion, mainMenuButtonAbout, mainMenuButtonEval, mainMenuButtonQuit]
    mainMenu = Menu("Main menu", mainMenuButtons)
    exitFlag = False
    functions = [showName, showVersion, showAbout, runCommand, stop]

    try:
        while not exitFlag:
            val = mainMenu.display()
            if val is not None:
                if val is 0:
                    functions[-1]()
                else:
                    functions[val-1]()
    except KeyboardInterrupt as e:
        print('Got execption {0}... exiting now'.format(e))

if __name__ == "__main__":
    main()