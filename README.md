# SciRoc 2021 - Object Detection Module

## Authors

> Marco Gabriele Fedozzi <5083365@studednti.unige.it>  
> Francesco Ganci <4143910@studenti.unige.it>  
> Laura Triglia <4494106@studenti.unige.it>  

## Interface Action Server

```
# goal
# possible modes:
## 0 -> Enumeration
## 1 -> Classification
## 2 -> Comparison
uint8 mode
string[] expected_tags
string table_id
---
# result
string[] found_tags
bool match
---
# feedback
string step
```

Where "mode" in the goal is an expression of what the interface AS should do, eg.
- "enumerate"
- "classify"
- "compare"
To which the module will respond by executing calling different actions served by other ObjDet components.

## table_id

By adding here the name of one of the expressed POIs, the module will retrieve the 2D position of such point, and send it to the inner action servers to make TiaGo look towards the table.
