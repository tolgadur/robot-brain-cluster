import smach

import sm_states

# Defining states
IDLING = 'idling'
HOTWORD_RECOGNITION = 'hotword_recognition'
TRANSITION = 'transition'
LISTENING = 'listening'
FACE_TRAINING = 'face_training'
TURNING = 'turning'
INTERPRETATION = 'interpretation'
RECOGNITION = 'recognition'
INFORMATION = 'information'
MESSAGE_DELIVERING = 'message_delivering'
AUDIO_OUT = 'audio_out'
END = 'end'


def init_sm():
    """
    @brief: initialize the state machine
    @return - smach.StateMachine: the state machine
    """
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=[END])

    # Open the container and initialise it

    ############################# LINEAR ######################################
    with sm:
        smach.StateMachine.add(IDLING, sm_states.Idling(),
                               transitions={HOTWORD_RECOGNITION: HOTWORD_RECOGNITION}
                               )

        smach.StateMachine.add(HOTWORD_RECOGNITION, sm_states.HotwordRecognition(),
                               transitions={LISTENING: LISTENING},
                               remapping={'lang_code_in': 'lang_code_out'}
                               )

        smach.StateMachine.add(LISTENING, sm_states.Listening(),
                               transitions={INTERPRETATION: INTERPRETATION},
                               remapping={'lang_code_in': 'lang_code_out'}
                               )

        smach.StateMachine.add(INTERPRETATION, sm_states.Interpretation(),
                               transitions={RECOGNITION: RECOGNITION,
                                            LISTENING: LISTENING,
                                            HOTWORD_RECOGNITION: HOTWORD_RECOGNITION,
                                            FACE_TRAINING: FACE_TRAINING},
                               remapping={'sentence_in': 'sentence_out',
                                          'lang_code_in': 'lang_code_out'}
                               )

        smach.StateMachine.add(RECOGNITION, sm_states.Recognition(),
                               transitions={INFORMATION: INFORMATION,
                                            MESSAGE_DELIVERING: MESSAGE_DELIVERING},
                               remapping={'action_in': 'action_out',
                                          'lang_code_in': 'lang_code_out'}
                               )

        smach.StateMachine.add(INFORMATION, sm_states.Information(),
                               transitions={TURNING: TURNING},
                               remapping={'recog_elements_in': 'recog_elements_out',
                                          'action_in': 'action_out',
                                          'lang_code_in': 'lang_code_out'}
                               )

        smach.StateMachine.add(FACE_TRAINING, sm_states.FaceTraining(),
                               transitions={FACE_TRAINING: HOTWORD_RECOGNITION},
                               remapping={'lang_code_in': 'lang_code_out',
                                          'name_in': 'name_out'}
                               )

        smach.StateMachine.add(TURNING, sm_states.Turning(),
                               transitions={HOTWORD_RECOGNITION: HOTWORD_RECOGNITION},
                               remapping={'lang_code_in':'lang_code_out'}
                               )

        smach.StateMachine.add(MESSAGE_DELIVERING, sm_states.MessageDelivering(),
                               transitions={HOTWORD_RECOGNITION: HOTWORD_RECOGNITION,
                                            RECOGNITION: RECOGNITION},
                               remapping={'receiver': 'receiver',
                                          'message': 'message',
                                          'recog_elements_in': 'recog_elements_out',
                                          'lang_code_in': 'lang_code_out',}
                               )



        return sm
