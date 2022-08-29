
## Expectations for faculty supervisors

Faculty supervisors are so named because they play a role similar to many engineering supervisors in industry. Engineering supervisors are available for questions during uncertainty or lack of direction, they provide helpful criticism of work (both to the engineer and the management team), and they will sign off on the work of the junior engineer before their designs are released for build.

The business unit manager is left to worry about the time line of projects, if the engineer is showing up to work, if they are following policies, etc. Importantly, the engineering supervisor isn't the manager of the junior engineer. The engineering supervisor is a senior engineer with expertise in the area that can be a resource for the current project and facilitate the development and assessment of the junior engineer(s). 

Faculty members take on the engineering supervisor role for the capstone team. They should not take on a management role for the team. 

### Specifics of what the role entails

#### Scope

Each project starts with the team writing a detailed project proposal. The project proposal contains many things and asks the students to think and write about many things. However, for the faculty supervisor, the most important thing to focus on is the scope of the project. Is the scope appropriate? The faculty supervisor should provide input to the team regarding the scope and help them to adjust appropriately. 

Some teams will attempt to "low ball" the scope. That is, they try to set the expectation low so that meeting the project objectives is easy. In this case, the faculty supervisor should provide feedback to the team that the scope is not significant enough. The faulty mentor and the instructor can work with the team to improve the scope by adding objectives to the scope. 

More often, teams will have great ambition that exceeds what is realistically possible. This problem often coincides with a vague project. ie. solve world hunger. Here, the faculty mentor should push the team to think about the project more critically to arrive at a well defined project with an appropriate scope. 

#### Conceptual Design

During the conceptual design phase, the team will begin to decompose the project into subsystems and identify areas of risk (subsystems in which they have lower confidence).  The team will also begin to construct a timeline of how to proceed with the project. 

The faculty mentor should evaluate whether the planned system is well defined and appropriate. With well defined and appropriate meaning the team has developed a concept and plan for the coming design which is easily understood by the mentor and that what they intend *could* (in the opinion of the mentor) meet the objective.

#### Detail Design and Signoff 

During the detail design phase of the project, team members will begin working on producing schematics for each of the subsystems that were described in the conceptual design. For each of the subsystems, the team must write a signoff document and submit it to their faculty supervisor. 

Essentially, the signoff document will inform the faculty mentor of the complete expectations (specs) the team has for what the subsystem will do. A *buildable* schematic will be included. The design will have been relevantly analysed using principles of electrical engineering (or physics et al as appropriate) to show that the design should meet the expectations. A BOM will be included in the signoff which includes all components that will be used.

##### Reasons to not sign off

- if the specs are vague
- if the specs are incomplete in the opinion of the faculty member (the supervisor is the subject matter expert, so if they feel the specs are incomplete, they are)
  
- if the schematics are not appropriate (circuits require circuit diagrams of some variety, mechanical systems require 3d models, etc)
- if the schematics are not detailed enough to be buildable by a 3rd party
  
- if the analysis does not substantiate that all the specs will be met
- if the analysis fails to consider important aspects of the system (ie. a power bus which has been analyzed to ensure that it provides the appropriate current and voltage but the noise from switching power supplies and motors et al has not been analyzed to verify that it is within the specs set by the components connected to the power bus)
- if the analysis is inappropriate. ie. an analysis has been performed to substantiate that the specs are met, but the analysis was done incorrectly or the wrong analysis was used. 
  
- if specs are assumed to be met without clear analytical reasoning. (this happens often with evaluating the analog to digital conversion resolution or the sampling rate of sensors. students often know that sensors should sample at the nyquist rate, but they have difficulty knowing how to calculate the nyquist rate in more abstract scenarios. ie. if attempting to measure the acceleration experienced by a cyclist, what is the nyquist rate of the signal?)
  
- if the BOM is not given with all components and prices (sometimes students will choose inappropriate components. If you happen to see an inductor that costs $75 without a good reason, this should not be allowed.)

##### When to signoff
If a buildable schematic is submitted for a design with the complete specs clearly given and shown to be met based on clear and convincing analysis of the design. And, if the BOM looks appropriate. 

##### Other things to look for
A common mistake made by teams is to treat each individual component as a subsystem. However, this is not generally acceptable. There are cases where this can make sense but only if they have identified a single component that truly does provide all the responsibilities of a subsystem defined in their conceptual design.

Treating components as subsystems wastes the time of the faculty member and makes the analysis trivial.


##### The purpose
What engineers learn in their discipline is not without purpose. When they begin to work for an engineering firm, the firm will expect that the engineer will be able to design and analyze those designs (with some help and guidance by other more senior engineers). This ensures that money is not wasted on designs that were doomed from the start (lowers the risk incurred by the firm).

Similarly, by requiring that students analyze the designs in this manner, they will identify many of the problems early. And they will learn how to apply course material in a realistic context. 

##### What not to do
Do not attempt to make sure that the design will work. By requiring the students to analyze the designs and holding them to a high standard, the probability that the design will work is greatly improved. However, it is likely that the design will still not work. This is good. Students will learn even more by having some success and some failure. When they have analyzed something and expect it to work, they will develop a deeper understanding still by having to analyze why they were wrong. Further, it provides an opportunity for the students to improve the design (which is an important part of the engineering process). 

Faculty supervisors should provide a high level of critical input into the specification and analysis of a designed system, and to recommend improvements to the design. However, it is not the faculty members role or responsibility to ensure that the design will work. 

#### Prototype 
When the students have completed the design and have built the minimally functional prototype, they are to demonstrate the prototype via video to the faculty supervisor. A minimally functional prototype integrates all the subsystems and components but with diminished functionality. The functionality should be relevant to the purpose. ie. a gps connected to a microcontroller that is sending garbage data is minimally functional while a gps and microcontroller that blinks a light is not.

Judging the prototype acceptibility is highly subjective. The purpose of this milestone is to have a waypoint between build start and complete functionality. Students are to negotiate before hand with the supervisor regarding what is acceptable for the prototype.

#### Experimentation
Students must conduct statistically significant (ie. N>1 and N appropriate to the context) experimentation to show that measures the attainment of all the design specs. It is not necessary that all specs are met. Rather, the team must measure them to identify what worked and what didn't (and why) and to identify areas of necessary improvement. 

When experimentation is complete, the list of improvements should be developed in conjunction with the supervisor. The team must negotiate with the supervisor to identify the necessary improvements. 
