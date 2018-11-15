# Regarding the word parsing program

| Tag | Meaning |
| ------ | ------ |
| VS | Simple action (Save person) |
| AV  | Auxiliary verb, ignored if more than 2 verb in sentence|
| VG | Movement action, place required |
| VGO | Movement action requires object |
| VO | Action regarding an object, object required, place optional |
| KO | Known Object |
| LO | Location |
| CJ | Conjunction (and/or) |
| AD | Adverb  (down,fast,upstairs,out)
| PP | Personal Pronoun |
| KP | Known Person |
| UW | Unknown Word |

| Tag | Method and parameters |
| ---- | ---------------------|   
| VG  | goTo(LO) |
| VS  | hard coded method required |
| VO  | take(OD), putDown(OD)      |
| VGO  | carryToLocation(OD,LO) |
