import smach

class ErrorHandlingState(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['error_handling_aborted','try_again'])
        self.retry_counter=0
    def execute(self,userdata):
        if self.retry_counter<2:
            self.retry_counter=self.retry_counter+1
            return 'try_again'
        else:  
            self.retry_counter=0          
            return "error_handling_aborted"
