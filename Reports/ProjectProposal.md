           

## This Document - Project Proposal Specification

This document is not an outline for the project proposal. This is an explanation of what a project proposal is intended to be. It is up to each team to understand the role the project proposal plays and to construct their proposal appropriately. There are some requirements given herein that **must** be met. However, not all required items are marked by a "**must**". Further, a project proposal that attempted to only include these requirements will not be graded favorably. This document is detailed and should be read with a goal of understanding the guiding principles rather than simply a list of expectations.

Regarding examples that are given, keep in mind that they should not be considered complete. That is, if an example is given for the objective, you should not think that this example would suffice for the entire objective section.

In the sections that follow you will find all the information that is necessary to understand the expectations for the project proposal.

### The Project Proposal

In this section, an overview of the broad goals for the project proposal is discussed. In the subsections that follow you will find some specific requirements regarding format and team member contribution.

The goal of a project proposal in general is to convince someone/some organization that a project is worth undertaking and (typically) that you are the right person/group to undertake the project.

So, in the broadest sense, a project proposal is persuasive writing. However, in this context persuasion does not primarily rely on an argument. Rather, persuasion is accomplished by formulating the problem, measures of success, and feasibility in a manner that considers resources and unintended consequences. 

A good project proposal will leave the reader clearly understanding

1. the identified and formulated objective (Often this is some problem to be solved and the relevant specs and constraints) 

2. the background (Background includes everything necessary to understand: what the objective is specifically, why the objective is important, what will be required to address or solve the objective, how success will be measured)

3. the unknowns and obstacles that may interfere with the success of the project and how the team intends to address them.

4. the measures of success that will be used, why they are appropriate, and how the measurement will be made.

5. the things which are anticipated to be needed for the completion of the project (detailed consideration of components, testing machinery, skills, et al.).

6. the anticipated timeline (detailed outline of how long the project is likely to take based on the tasks to be completed and the personnel available to complete them).

7. the broader impacts (how will the project and its completion impact the society, economy, environment, culture, public health and safety, et al.).

#### Required Format and Style

##### IEEE Conference Format

The project proposal **must** be completed using the latest revision of the IEEE conference, 2 column format. There are templates available through IEEE for both latex and word.

Overleaf is a free, online latex editor and compiler that facilitates collaboration. However, using overleaf is not required.

##### Citations

Use the IEEE citation inline method and bibliography style. All sources used to lend information to the project proposal **must** be cited. Many citations are expected, as without citations information that is not common knowledge to those in ECE seems to be the opinion of the team members rather than a substantiated fact.

##### Writing Point of View

It is more professional to use third person. However, there are times when deviating from third person improves clarity and readability.

As an example: Each team should primarily write in third person unless one finds oneself in a situation which is clearly better suited to first person.

Better: Each team should primarily write in third person unless you find yourself in a situation which is clearly better suited to first person.

Best: Each team should primarily write in third person unless a specific situation is clearly better suited to first person.

#### Required Contribution from Each Team Member

It is up to each team to designate tasks and work together toward the completion of the capstone project. However, each team member must contribute meaningfully to each part of the project. Each team member **must** contribute meaningfully to the writing of project proposal. The work of each team member toward the completion of the project proposal **must** be clearly documented in the individual journal entries.

### The Introduction

The introduction **must** be the first section in a proposal (and any paper in general). The introduction is the "elevator pitch" of the entire project. The explicit purpose is to introduce the objective, why it is important (briefly), and what you propose to do about it. In some cases, the introduction of a paper may be all that a reader sees (often by their own choice). So, the introduction should get the reader's attention and convince them that the rest of what you have written is important and worth reading.

#### Helping the Reader Know What to Expect

Keep in mind that your readers may not have access to the document you are currently reading. So, toward the end of the introduction, it is best to include a subsection specifically to give the reader some sense of what the proposal is going to tell them moving forward. Then, in each of the sections in the proposal you should tell the reader what to expect from the given section somewhere near the beginning. This isn't a hard rule but is a best practice. In general, this is done at the beginning of sections but not subsections as they are contributors to what you have already said the enclosing section is going to do. 


 ### Formulating the Problem

Formulating a problem/objective means to clearly define the objective through the inclusion of background information, specifications, and constraints. A (potentially) helpful analogy would be this: formulating a problem is the process of using background information, specifications, and constraints to "fence in" the objective, so that it is unambiguously clear what is and is not being addressed and why.

As an example, a past project's objective was to build a telemetry system that would function at the end of an oil drilling tip. This statement identifies the objective, but it is not formulated. 

- Why do we want something that does that?

- What is so hard about this that it requires a dedicated, multi-person engineering project?

- Why can't we use something off the shelf?

The background information provides context and detail which helps to answer questions like those above and allows the team to define the boundaries of the problem (objective).



##### A Quick Example: Including background information

Oil drills don't go straight down because as they go past layers of rock the drill may follow a path of least resistance, so it is necessary to monitor the direction of the drill (This piece of background helps the reader to understand why a drill telemetry system is needed). Monitoring the direction is made difficult by the fact that the rock through which the drill passes is often magnetic, thus preventing magnetic sensing of direction (The more relevant background that is included, the more clearly the problem can be defined). Worse, the drill will generate significant heat (over 150C) at the tip which is where the telemetry system must be located. The drill will also produce continuous vibration that may interfere with the electronics. Therefore, typical electronics wouldn't be suited to the environment.

The problem is further complicated by the need to send information from the tip of the oil drill to the surface will be a non-trivial task due to the depth in the earth, magnetic interference, and heat. Wires running from the tip to the surface will melt or break and wireless transmission is impractical due to the environment. The conventional solution for sending signals from the tip on most oil drills is a method based on changing the rate of runoff flow. As the drill works, it generates runoff. This runoff is pumped to the surface. So, the inlet to the pump can be adjusted to restrict or augment flow. Restricted flow is analogous to a zero and augmented flow is analogous to a one. This method of communication creates a very low speed of data transmission, making it difficult to know the drill position in close to real time. If the positioning information lags, the position control system may become unstable.



#### Specifications and Constraints

Take the time to present the specifications clearly and concisely, referencing the background material briefly as needed for explanation. 

##### Specifications

The specifications are traits imposed by stakeholders for the project to meet their requirements. Stakeholder specifications constrain the solution and provide required expectations. Through the course of the project, it may become clear that a specification is not attainable. If this occurs, discussion with the stakeholders on how to compromise is necessary.

As an example, the specifications for the oil drill telemetry system were that the system must fit on a commercial oil drill, be able to operate in the typical environment, and provide accurate location information quick enough for the direction control system to maintain the desired drilling direction. 

All of the specifications given are very fuzzy. It is the engineer's job to take a fuzzy specification and distill it into something more useful. For instance, through background research it can be determined that drill tips are built to be space efficient to limit the diameter of the hole that must be drilled. Therefore, the size of the drill tip storage compartment is very limited. In the target drill the compartment is 1 inch wide. 

Verbal spec: fit in the tip of the target drill (fuzzy)
Formulating the spec using background info: the system shall be less than 1 inch in diameter such that it fits in a 1 inch cylinder (measureable)

So, using background information, engineers are able to formulate fuzzy, verbal specifications into precise constraints.

For further information about this refer to Chapter 4.1 and 4.2 of the Nasa Systems Engineering Handbook. 4.1 deals with eliciting stakeholder expectations. The next section (4.2) deals with constructing "shall" statements (like the one above) and building technical requirements from the stakeholder expectations.

##### Other sources of constraints: Standards

The stakeholders are not the only source of constraints for the project. Many will be imposed by governing bodies and standards organizations. Perhaps to make a device that operates in certain locations, the government may require the components which are included in the construction be CE certified. This would then be a standard that constrained the solution which is to be designed.

As an example of a standard which would impose a constraint, if Wi-Fi is to be used, there are specific standards that govern the operational frequency of the protocol. The engineering world is full of standards and it is the responsibility of the engineer to seek them out and adhere to them. Therefore, each team must search out all standards which apply to the project and ensure that the design conforms to the standard. 

Every industry and project type has many types of standards. These range from the most obvious (the national electric code) to the obscure (ISO 18646:2016 - locomotion for wheeled industrial robots). It is the responsibility of the engineer to become aware of the applicable standards and to constrain the project appropriately so that standards are met.


##### Other sources of constraints: Externalities

It is obviously important to consider the direct/intended implications of the work to be completed. ie. How would a better oil drill telemetry system affect the drilling industry? But it is also important to consider the externalities. That is the impact that the engineering, manufacturing, or final product may have on public health, safety, welfare, as well as global, cultural, social, environmental, and economic factors.

These broader impacts will again be discussed in the path to solution section. However, here the focus is on identifying ways which these externalities should impose constraints on the project.

As an example, perhaps there are two sensing devices that could be used to detect a specific signal in an aquatic environment. However, one of the sensors has a tendency to leach lead as it ages. In the long term the sensor that introduces heavy metals into waterways will have negative impacts on the environment and public health and safety. So, it would be appropriate to constrain the design so that all components are either environmentally safe for long term submersion or sealed to prevent leakage.

Design should always be done in a manner that considers potential unintended consequences and is constrained to minimize the potential risks.

##### A little more about constraints

Constraints (regardless of their origin) must be unambiguous and impose measureable requirements on the design. At times, conforming to the constraints can be difficult. However, they are beneficial in that they provide clear expectations. When considering what makes an engineering project a success, the most important factor is whether or not the specifications and constraints were met.




#### Survey of Solutions

As a part of the proposal each team should research what is already available in the research literature, on the market, and in the industry. Then those findings should be curated and presented. The information should not be presented idiosyncratically. Rather, you should make sense of it and present it in a digestible manner utilizing subsections as appropriate.

Other solutions to similar problems often are a great source of information about what sort of background information and standards apply. If a project were to seek to build an RFID reader, it would be helpful to look at the list of standards with which existing RFID readers comply. Further, reading a few research papers on RFID systems would yield helpful insight into the background information and the associated challenges. 

Remember, it is required that any information which is not common knowledge (taught in an ECE course) be cited. 

#### Summarizing the Problem

In this subsection you should draw on the information already given in the background, specs and constraints, and the survey of existing solutions to summarize why the objective is important, non-trivial, and distinct.

Often what makes a project different from other available solutions is the set of specifications. Even though a commercial solution may exist, it may be too costly. And it may be a specification that the total cost be kept below a certain number. Similiarly, the function of existing solutions may not meet the stakeholder specifications.

### Looking Down the Path Toward Solution

Now that the problem has been clearly identified and formulated with all specifications and constraints, observing all relevant standards and considering the externalities, it is possible to consider what a solution may look like. To explain the potential solution, provide all relevant science, engineering, and mathematical detail so that the reader can clearly understand how the solution may work.

This section is not intended to be a full explanation of a proposed solution (If you already had a full solution, the project is probably not necessary). Rather, it provides the skeleton of what a solution will likely involve as seen from early on. Returning to the oil drill example, if we rule out magnetic positioning, research quickly yields two alternatives: accelerometer detection of gravity or gyroscopic detection of rotation. So, from the outset it is apparent that the solution will likely involve one of these solutions. For the reader to have confidence in these potential solutions, detail should be given as to how each of these could contribute to a solution from a science, engineering, and math perspective.

#### Unknowns/obstacles, implications, necessary experiments

In most projects there are unknowns. These pose a risk to the potential success of the project. As an example, perhaps the company which funded the oil drill telemetry system could not give vibration magnitude or frequency information for how the oil drill will shake. If this is the case, how can the team be certain that the electronics can function in the environment? They can't without first understanding the vibrations.

Often, unknowns require experimentation to form a clearer image of what will be required. Sometimes typical lab experiments are viable. Other times, it may be necessary to utilize modeling and mathematics to make conservative estimates. The team should identify unkwnowns and suggest experiments, simulations, and analysis which can address these unknowns.

Further, each team should consider how these unknowns could affect the solution. As an example, it may be that after simulation and analysis it is found that the oil drill is capable of producing vibrations which are greater than what any electronics can withstand. Then it may be necessary to amend the specifications appropriately.

#### The Measures of Success

The last component to formulating an image of the solution is to explain how the constraints (the things required for success) will be evaluated. It is insufficient that the original design was intended to produce a system which is capable of meeting the constraints. A method for experimentally validating that the constraints are met must be employed. As with any experiment, the experimental design (what you intend to do and how you intend to do it) must be statistically informed (getting the desired result once does not prove much). 

In the Nasa Systems Engineering Handbook, this is discussed in section 4.2. They refer to the measures of success as measures of effectiveness (MOE).

Each team is to explain the experiments that are to be employed to establish success and why they are appropriate. 

As a note, would it be ethical to claim that a solution met specifications without any validation?

#### Broader Implications, Ethics, and Responsibility as Engineers

It is important that each team consider the impact that a solution may have in global, economic, environmental, and societal contexts. It is impossible to predict all consequences. However, failing to consider possible externalities at all leads to major issues. As an example, theoretically, if the oil drilling process were largely random (due to poor control of the drill) and it employed many people, then perhaps communities that were largely employed by oil companies as drilling workers would be negatively impacted if the process were made more efficient. Negative impacts should be identified and a plan for mitigation should be proposed if it is impossible to prevent the potential negative.

The consideration of broader impacts flows naturally from the expectation that each engineer will conduct their work ethically. Significant consideration should be given to what ethical considerations are involved in and what each individual's responsibility as an engineer imposes on this project. These responsibilities and ethical considerations **must** be detailed appropriately.

### The Resources

Each project proposal **must** include a detailed explanation regarding what resources will be necessary and a budget proposal with justification. The resources include, but aren't limited to, necessary software, equipment, components, testing machinery, and prototyping costs.

#### Personnel

Each team should also consider the team members and the skills which are represented. This should be compared to the skills expected to be necessary to complete the project. Some skills will be important for each team member to possess (Example: soldering) while other skills may only be needed by a single team member (Example: 3D printing). Identify skills that must be obtained. Then, provide a plan by which the team intends to close those skill gaps.

Do not make the mistake of expecting that the team already knows all the relevant theoretical and technical knowledge necessary to complete the project. This is almost never the case. Even if the task were to design a simple amplification circuit, doing so in practice is always more difficult than expected and typically requires deepening the team's existing understanding.

#### Timeline

The last component of a proposal is the proposed timeline. This timeline should be detailed, giving all the important deadlines and tasks to be completed. The timeline should also establish a viable task assignment. This does not mean that the team member listed with the task at this point will definitely complete that task. This timeline is only intended to substantiate that the team has the personnel to complete the set of tasks by the necessary dates. Many specific tasks will likely be unknown at this point. However, those tasks will likely fall into broad categories that will be able to be foreseen.

The timeline should include a professional looking Gantt chart showing the team members, their tasks, and the completion dates. It should also show the milestones (Thanksgiving break, Christmas break, Spring break, and end of the semester).

If it becomes apparent that it is not possible to meet the dates, it will be necessary to limit the scope of the project and appropriately target a subset of the specifications. Then it would be necessary to update the project proposal to reflect the personnel constraint. The process of building the proposal (and the design, later) is extremely iterative.

### Rubric

Was the project proposal written without grammatical errors? (TA) (-1 per or -5 max) 

Was first person perspective used where not appropriate? (TA) (-1 per or -5 max) 

Was the project proposal submitted late? (TA) (-4 per week) 

Was a proposal submitted? (TA) (+10)

Was the project proposal submitted in IEEE format? (TA) (+7) 

Was the project proposal submitted as a pdf? (TA) (+5) 

Does this proposal clearly present the elevator pitch of the project (persuasive summary) in the introduction? (coordinator) (+8 max) 

Does this proposal clearly state the objective/problem? (supervisor) (+7 max) 

Does this proposal sufficiently discuss the background information necessary to understand any and all objectives, specifications, constraints, context, desired outcomes, and measures of success? (supervisor) (+10 max)

Does this proposal clearly formulate and consider all constraints arising from stakeholders, standards, and broader considerations? (supervisor) (+10 max) 

Does the proposal significantly discuss existing solutions and relevant literature? (supervisor) (+7 max) 

Is the formulated problem summarized well? (coordinator) (+3 max) 

Does the proposal consider unknowns and sources of risk and how to address them? (coordinator) (+5 max) 

Does the proposal consider the broader implications? (coordinator) (+7 max)

Does the proposal develop appropriate measures of success that are justified based on the constraints? (supervisor) (+7 max)

Is the feasibility of the project considered (Resources, personnel, and timeline)? (coordinator) (+7 max) 

Does the project proposal work together to establish that the project (objective, background, desired outcomes, measures of success, required resources, timeline, feasibility, and broader impacts) is worth undertaking and that they are the right group to undertake the project? (supervisor) (+7 max) 


### Revision

The environment in this class is intentionally geared to be similar to industry experience, though not identical. So, a grade will be calculated. You are to address the issues that are identified and re-submit to regain **up to** 50% of the lost points. Changes to the document must be in a different color otherwise they will not be counted toward the improvements.

### Feedback in ilearn

Each team has 1 week from the date the feedback is given to address the problems identified and resubmit to reclaim up to half of the lost points.
