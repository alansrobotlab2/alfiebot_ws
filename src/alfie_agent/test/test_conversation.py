"""Conversation words: backchannels, closers, stoppers, transcript wake, echo."""
from alfie_agent import conversation as c


def test_backchannels_are_not_turns():
    for t in ("Yeah.", "Okay.", "Mm.", "Yeah, that's great.", "Good, Alfie. Thank you.",
              "okay cool thanks"):
        assert c.is_backchannel(t), t


def test_requests_and_questions_are_not_backchannels():
    for t in ("Yeah, check the other one.", "Is that right?", "Okay, pick up the can.",
              "What time is it?"):
        assert not c.is_backchannel(t), t


def test_after_a_question_yeah_is_the_answer():
    assert not c.is_backchannel("Yeah.", after_question=True)
    assert not c.is_backchannel("Sounds good.", after_question=True)
    assert c.is_backchannel("Mm.", after_question=True)


def test_closers_and_stoppers():
    assert c.is_closer("That's all, thanks.")
    assert c.is_closer("Thanks, Alfie.")
    assert c.is_closer("Okay, never mind, Alfie.")
    assert not c.is_closer("Thanks.")           # a backchannel, not a goodbye
    assert not c.is_closer("Stop the music.")   # a request
    assert c.is_stopper("Alfie, stop.")
    assert c.is_stopper("Hold on.")
    assert not c.is_stopper("Stop the music.")


def test_transcript_wake():
    assert c.split_wake("Hey Alfie, what time is it?") == "what time is it?"
    assert c.split_wake("Alfie.") == ""
    assert c.split_wake("Um, Alfie, dance!") == "dance!"
    assert c.split_wake("Hey Alfy what's up") == "what's up"   # ASR spelling
    assert c.split_wake("So I told Alfie about it") is None
    assert c.split_wake("Uh um Alfie hello") is None          # two fillers
    assert c.split_wake("What time is it?") is None


def test_a_mention_is_not_an_address():
    assert c.mentions_name_midsentence("Did Alfie finish the report?")
    assert c.mentions_name_midsentence("I told Alfie already.")
    assert c.mentions_name_midsentence("Alfie's battery is low.")
    assert not c.mentions_name_midsentence("Alfie, what time is it?")
    assert not c.mentions_name_midsentence("What time is it, Alfie?")
    assert not c.mentions_name_midsentence("What time is it?")


def test_echo_overlap():
    said = "The red can is on the kitchen table next to the window."
    assert c.echo_overlap("the kitchen table next to the window", said) == 1.0
    assert c.echo_overlap("no wait pick up the blue one", said) < 0.6


def test_echo_needs_a_contiguous_fragment_of_the_reply():
    said = ("Once upon a time, a small robot named Alfie lived in a workshop. "
            "It is half past five.")
    assert c.is_echo("a small robot named Alfie lived", said)
    assert c.is_echo("half past five", said)
    # The user reusing scattered words of the reply is not echo.
    assert not c.is_echo("actually what time is it", said)
    assert not c.is_echo("what time is it", said)
    # Stop and wake words never are.
    assert not c.is_echo("stop", "I will not stop now.")
    assert not c.is_echo("Alfie, stop.", "Alfie, stop.")
    assert not c.is_echo("Alfie.", "My name is Alfie.")
    # Echo of a reply that names him still counts.
    assert c.is_echo("Alfie lived in a workshop full of tools", said + " full of tools")


def test_question_detection():
    assert c.ends_with_question("Want me to grab it?")
    assert c.ends_with_question('He said "really?"')
    assert not c.ends_with_question("I grabbed it.")
