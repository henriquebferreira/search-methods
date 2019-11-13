# Search methods

This mini-project was developed for the Artificial Intelligence and Decision Systems course at Instituto Superior Técnico (IST). It addresses the problem of scheduling the launch of components for the in orbit assembly of a large structure. Given a set of components to be launched, a launch timeline and costs, and a construction plan, the goal is to determine the assignment of components to launches, such as the total cost is minimized. Read the [assignment description](assignment-details.pdf) for more details.

Two search methods are available with the objective of finding a solution. The search method is specified by a flag as a command line argument:
* (-u) uninformed search method - Uniform Cost Search
* (-i) informed search method - A*

The filename containing the problem specification is also a command line argument.

## Running the tests

For instance, to test the informed search method given the input present in the [mir](tests/mir.txt) file one may run 
```
python solver.py -i tests/mir.txt
```

## Authors

* **Henrique Ferreira** - [GitHub](https://github.com/henriquebferreira)
* **Manuel Rosa** - [GitHub](https://github.com/ManuelDCR)

## License

This project is licensed under the BSD 3-Clause "New" or "Revised" License - see the [LICENSE.md](LICENSE) file for details
