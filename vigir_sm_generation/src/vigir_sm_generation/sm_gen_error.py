
class SMGenError(Exception):
    """
    This class represents errors that may occur while trying to generate state
    machines.
    """
    def __init__(self, error_code):
        """ Initialize an error with a given BSErrorCodes. """
        self.error_code = error_code

    def __str__(self):
        return "BSErrorCode: {0}".format(self.error_code)
