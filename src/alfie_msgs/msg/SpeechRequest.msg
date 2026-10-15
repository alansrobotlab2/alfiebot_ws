string text
uint8 volume 100
# Agent turn this clause belongs to (0 = untagged). A barge-in drops queued and
# late-arriving requests from turns at or below the last one it interrupted.
uint32 turn_id 0
# True while more clauses of the same reply are coming: TTS keeps `speaking`
# high between them. An empty `text` with more_follows=false ends the reply.
bool more_follows false
