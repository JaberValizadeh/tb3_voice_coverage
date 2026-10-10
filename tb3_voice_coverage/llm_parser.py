#!/usr/bin/env python3
"""
Turn a free-form spoken request into a validated cleaning command.

The keyword parser in voice_coverage_node only matches a fixed 26-word
vocabulary: you must say almost exactly "clean the kitchen". This module lets
the same robot handle how people actually talk -

    "tidy up where I sleep"                  -> bedroom
    "do everywhere except the study"         -> all rooms but the study
    "the room with the bath, then the loo"   -> bathroom
    "آشپزخانه را تمیز کن"                     -> kitchen

and ask a question back when the request is ambiguous.

Two rules keep the LLM from being a liability:

1. **Never trust the room names it returns.** Everything is filtered against
   house_layout.TOUR, so a hallucinated room cannot reach the planner.
2. **Never depend on it.** Any failure - no config, no network, bad JSON,
   API error - falls back to the keyword parser, so the package keeps working
   offline. The LLM is an upgrade to understanding, not a new requirement.

Model credentials use the same AutoGen-style OAI_CONFIG_LIST.json that
tb3_llm_control already uses, so there is one place to keep keys.
"""

import json
import os

from tb3_house_full_coverage import house_layout as L

ACTIONS = ('clean', 'stop', 'pause', 'resume', 'status', 'clarify')

# Plain-language hints so the model can resolve descriptions ("where I sleep")
# rather than only exact room names. Keys must match house_layout.TOUR.
ROOM_HINTS = {
    'kitchen': 'the kitchen - where food is cooked',
    'living': 'the living room, lounge, sitting room, front room',
    'store': 'the small store room, storage room, pantry, box room',
    'bathroom': 'the bathroom, toilet, loo, washroom, shower room',
    'bedroom': 'the bedroom - where people sleep',
    'study': 'the study, office, where the desk is',
    'corridor': 'the corridor, hallway, passage, landing',
}


def build_system_prompt():
    rooms = '\n'.join('  %-9s %s' % (k, ROOM_HINTS.get(k, k)) for k in L.TOUR)
    return (
        "You turn a person's spoken request into a cleaning command for a "
        "robot vacuum in a house.\n\n"
        "The house has exactly these rooms. Use these exact keys:\n"
        + rooms + "\n\n"
        "Answer with JSON only, no prose:\n"
        '{"action": "clean|stop|pause|resume|status|clarify",\n'
        ' "rooms": ["room keys, in the order the person wants them"],\n'
        ' "reply": "one short sentence to say back to them"}\n\n'
        "Rules:\n"
        "- Only ever use the room keys above. Never invent a room.\n"
        '- "everywhere", "the whole house", "all rooms" -> every key.\n'
        '- Handle exclusions: "everywhere except the study" -> every key '
        "except study.\n"
        "- Keep the order the person asked for; if they gave no order, list "
        "them in any order.\n"
        "- If they name no room and it is not stop/pause/resume/status, use "
        '"clarify" and ask which room in the reply.\n'
        "- If they ask to halt, use stop. Hold on/wait -> pause. Carry on -> "
        "resume. How far along -> status.\n"
        "- Write the reply in the same language the person used.\n"
    )


def load_config(config_file, model):
    """Pick one entry out of an OAI_CONFIG_LIST.json by model name."""
    path = os.path.expanduser(config_file)
    if not os.path.isfile(path):
        raise IOError('no LLM config at %s' % path)
    with open(path) as f:
        entries = json.load(f)
    for e in entries:
        if e.get('model') == model:
            return e
    raise KeyError('model %r not in %s (have: %s)'
                   % (model, path, ', '.join(e.get('model', '?')
                                             for e in entries)))


def validate(raw):
    """Clamp whatever the model returned to something safe to execute."""
    action = str(raw.get('action', '')).strip().lower()
    if action not in ACTIONS:
        action = 'clarify'

    rooms, seen = [], set()
    for r in raw.get('rooms') or []:
        key = str(r).strip().lower()
        if key in L.ROOMS and key not in seen:   # hallucinations dropped here
            seen.add(key)
            rooms.append(key)

    if action == 'clean' and not rooms:
        action = 'clarify'
    if action != 'clean':
        rooms = []

    reply = str(raw.get('reply', '')).strip()
    return {'action': action, 'rooms': rooms, 'reply': reply}


class LLMParser:
    def __init__(self, config_file, model, timeout=20.0, logger=None):
        self._model = model
        self._log = logger or (lambda *a: None)
        self._client = None
        self._prompt = build_system_prompt()

        entry = load_config(config_file, model)
        from openai import OpenAI
        kwargs = {'api_key': entry.get('api_key', 'none'), 'timeout': timeout}
        if entry.get('base_url'):
            kwargs['base_url'] = entry['base_url']
        self._client = OpenAI(**kwargs)

    def interpret(self, text):
        """-> validated command dict. Raises on any API/parse failure."""
        r = self._client.chat.completions.create(
            model=self._model,
            messages=[{'role': 'system', 'content': self._prompt},
                      {'role': 'user', 'content': text}],
            response_format={'type': 'json_object'},
            temperature=0,
            max_tokens=200,
        )
        content = r.choices[0].message.content
        return validate(json.loads(content))


def to_command_string(cmd):
    """Canonical text the existing keyword executor already understands."""
    if cmd['action'] == 'clean':
        return 'clean ' + ' '.join(cmd['rooms'])
    return cmd['action']
