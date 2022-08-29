## This Document - Phase 1 Documentation Specifications

This document should not be thought of as an outline. Rather, this document explains the goal of the phase 1 documentation and specific things that the reader should understand after having read the phase 1 documentation. Armed with these expectations, each team is expected to design an appropriate document to accomplish the goal and appropriately educate the reader. 

### Big Picture Goal

The goal of the design phase 1 document is to leave the reader understanding the (1) fully formulated problem, (2) team's conceptual solution, (3) justification of the conceptual solution, (4) explanation of the path forward that will accomplish the detail design and analytical verification. 

DO NOT HAVE SECTIONS NAMED "justification of the conceptual solution" etc. the above is NOT an outline. It is the goal of the document. Consider all the requirements given herein and follow the process described in the "Writing Like an Engineer" document in the content section of ilearn.

### Specifications and Constraints for Design Phase 1

1. Must conform to the latest IEEE conference format.
2. All sources must be cited using IEEE inline citations unless the information is common knowledge among the writer's audience.
3. Must be in third person.
4. Must contain an introduction.
5. Must state the up to date fully formulated problem in a sub-section of the introduction.
6. Must contain an explanation of how this document fits into the larger process of engineering design.
7. Must contain a statement of ethical, professional, and standards considerations.
8. Must contain a complete, well specified system block diagram for the team's solution to the problem.
9. For each sub-system, the complete set of specifications (input, output, functionality) and constraints must be given.
10. For each sub-system, the analytical method (matlab simulation, spice simulation, nodal analysis, simulink simulation) which will be used to verify the specifications have been met must be given.
11. Must contain a gantt chart detailing **all** tasks that will be necessary to complete the detail design and analytical verification and by whom the tasks will be completed. (this includes any skill acquisition that will be necessary)


### Introduction

In this case, the goal of the introduction section is not to persuade. Rather, in a design document, the introduction is intended to reintroduce the fully formulated problem, summarize the phase 1 design, and set appropriate expectations. 

#### Restating the Fully Formulated Problem

The fully formulated problem is the objective with a summary of all specifications and constraints which completely disambiguate the objective. This was part of the project proposal. However, it may be that the specifications have changed in some way. So, state the fully formulated problem in the introduction of the design phase 1 document. For each of the specifications and constraints, explain the origin of the constraint/specification. The project **must** include 1 or more constraints/specs originating from each of the following: broader impacts, ethical considerations, and engineering standards. 

#### Summary

Summarizing a design document may seem difficult because it may be unclear what should be included in the summary. Rather than attempting to guess about this, consider the things that the design phase 1 document is expected leave the reader knowing. Any summary of the document should have the same goal(s).

#### Setting Expectations

In the following paragraph the shortcomings of a design are freely admitted so that the reader has appropriate expectations.

Oil drill telymetry is a difficult problem with many specifications and constraints. The conceptual design presented here complies with all the constraints. However, not all specified functions have been accomplished. The desired rate of communication is a difficult problem due to the need to only transmit at 10 baud and the large data structure to be sent. That being said, there are multiple possible avenues being considered which may provide a step toward solving this shortcoming.

The alternative, which engineers sometimes attempt, is to obfuscate the true state of affairs by writing in a manner that is intentionally unclear. Based on the IEEE code of ethics, is this something that is permissable?


### Ethical, Professional, and Standards Considerations

In the project proposal, each team was required to consider what the broader impacts of the project may be to the culture, society, environment, public health, public safety, and economy. They were also required to consider the standards organizations which would inform the design. Here, the updated ethical, professional, and standards considerations must be discussed. Further, each team must include a complete discussion regarding how these broader considerations have been used to inform your design (what constraints, specifications, or practices etc. are in place due to the consideration of these).


### Expectations for the Draft

The draft should show very significant progress toward completion of the design phase 1 document. There must be a complete outline in place with more than 60% of the document complete. The complete system block diagram for the conceptual solution must be in place with all specifications and constraints detailed for the sub-systems. 

The question "why?" will be asked very often during the review. Obviously, don't lie. However, it is expected that the design decisions will be justified (based on constraints, specs, broader considerations, availability, feasability, resources, etc.) rather than arbitrary!


### Block Diagram Expectations

Block diagrams are a great way to provide a big picture understanding of a system and the relationships of the individual components. In general, block diagrams borrow from visual modeling languages like the universal modeling language (UML). Each of the blocks represent sub-systems and each of the connections represent that the two (or more) blocks connected have a relationship. 

Each sub-system should be represented by a single block. For each block in the sus-system there should be a short explanation of the functional expectations for that block. For each of the connections, there should be a short description of the expectations for that relationship (method of communication, inputs, outputs, etc). 

The result should be a complete view of a well defined, complete system that delegates all atomic responsibilities to sub-systems and their interactions.


### Conceptual Design and Design Planning Objectives

1. Identify and minimize sources of risk
	1. critical unknowns
	2. delivery problems
2. Maximize attainment of stakeholder goals
3. Minimize consumption of resources
4. Optimize the Timeline for the Detail Design
	1. Address critical unknowns early
	2. If system $A$ places a constraint on system $B$, *in general* system $A$ should be designed first


### Rubric

Was the document written without grammatical errors? (-1 per or -5 max) 

Was first person perspective used where not appropriate? (-1 per or -5 max) 

Was the document submitted late? (-4 per week) 

Unprofessional communication. This does not include errors. This does include things that lack appropriate effort or care. (up to -3 per as appropriate)

Was a document submitted? (+10)

Was the document submitted in IEEE format? (+7) 

Was the document submitted as a pdf? (+5) 

Is there an introduction that reintroduces the fully formulated problem? (+10)

Is it clearly explained how this document fits into engineering design (I want to see that the students demonstrate some understanding of the engineering design process)? (+3)

Are ethical considerations discussed (evaluate if students demonstrate an ability to factor ethical considerations into engineering design considerations)? (+6)

Are standards addressed sufficiently (+6)

Is the conceptual solution explained in detail? (+10)

Is there a block diagram representing the complete conceptual solution? (+10)

Is the block diagram well specified? (every block and every line between blocks has associated expectations that are stated clearly and concisely and explained in the document) (+10)

Are there clear analytical methods described for testing each sub-system of the concept? (+5)

Are the next steps clearly explained (this may be via the gantt chart)? (+10)

Does the gantt chart map logically from the conceptual design? (+3)

Are all sources cited appropriately? (+5)
