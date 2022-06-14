import os

# AWK command to copy a file without the header line and get the maximum number of annotation in a view
# Variables have double $ because make escapes them
# Original command: awk 'NR>1 {m=$2;for(i=2;i<=NF;i++)if($i>m)m=$i;print $1"\t"m}'
extractBaselineFromAnnotationsCommand = "awk 'NR>1 {m=$$2;for(i=2;i<=NF;i++)if($$i>m)m=$$i;print $$1\"\\t\"m}'"

def integerProbabilityStr(probability):
    return '{:d}'.format(int(100.0 * probability))


def directoryContainingAnnotations(directory):
    return '../Phenotyping/{}'.format(directory)


def groundTruthFile(directory):
    annotationDirectory = directoryContainingAnnotations(directory)
    return '{}/annotations/ground_truth.csv'.format(annotationDirectory)


def directoryForResultFiles(directory):
    return 'tmp/{}'.format(directory)


def resultFile(directory, theta, probability, seed):
    resultDir = directoryForResultFiles(directory)
    probability = integerProbabilityStr(probability)
    return '{}/results_{:d}_{}_{:d}.csv'.format(resultDir, theta, probability, seed)


def annotationFile(directory, theta, probability, seed):
    resultDir = directoryForResultFiles(directory)
    probability = integerProbabilityStr(probability)
    return '{}/nb_annotations_{:d}_{}_{:d}.csv'.format(resultDir, theta, probability, seed)


def resultBaseFile(directory, theta, probability, seed):
    resultDir = directoryForResultFiles(directory)
    probability = integerProbabilityStr(probability)
    return '{}/results_baseline_{:d}_{}_{:d}.csv'.format(resultDir, theta, probability, seed)


def resultFileAggregateSeeds(directory, theta, probability):
    resultDir = directoryForResultFiles(directory)
    probability = integerProbabilityStr(probability)
    return '{}/results_{:d}_{}.csv'.format(resultDir, theta, probability)


def resultBaseFileAggregateSeeds(directory, theta, probability):
    resultDir = directoryForResultFiles(directory)
    probability = integerProbabilityStr(probability)
    return '{}/results_baseline_{:d}_{}.csv'.format(resultDir, theta, probability)


def resultFileAggregateProbabilitiesAndSeeds(directory, theta):
    resultDir = directoryForResultFiles(directory)
    return '{}/results_{:d}.csv'.format(resultDir, theta)


def resultBaseFileAggregateProbabilitiesAndSeeds(directory, theta):
    resultDir = directoryForResultFiles(directory)
    return '{}/results_baseline_{:d}.csv'.format(resultDir, theta)


def resultAllFileAggregateProbabilitiesAndSeeds(directory, theta):
    resultDir = directoryForResultFiles(directory)
    return '{}/results_all_{:d}.csv'.format(resultDir, theta)


def resultAllFile(directory):
    resultDir = directoryForResultFiles(directory)
    return '{}/results_all.csv'.format(resultDir)


def resultAllSortedFile(directory):
    resultDir = directoryForResultFiles(directory)
    return '{}/results_all_sorted.csv'.format(resultDir)


def agreementGraph(directory, theta):
    return '{}_agreement_{:d}.pdf'.format(directory, theta)


def agreementThetaGraph(directory):
    return '{}_agreement_theta.pdf'.format(directory)


def agreementProbabilityGraph(directory):
    return '{}_agreement_prob.pdf'.format(directory)


class MakeRule:
    def __init__(self):
        self.targets = []
        self.dependencies = []
        self.recipes = []

    def addTarget(self, target):
        self.targets.append(target)

    def addTargets(self, targets):
        self.targets.extend(targets)

    def addDependency(self, dependency):
        self.dependencies.append(dependency)

    def addDependencies(self, dependencies):
        self.dependencies.extend(dependencies)
    
    def addRecipe(self, recipe):
        self.recipes.append(recipe)

    def print(self):
        # Print the targets
        for target in self.targets:
            print(target + " ", end = '')

        # Print the symbol separating the targets from the dependencies
        if len(self.targets) > 1:
            print("&: ", end = '')
        else:
            print(": ", end = '')

        # Print the dependencies
        for dependency in self.dependencies:
            print(dependency + " ", end = '')
        # Print recipes
        for recipe in self.recipes:
            print("\n\t" + recipe + " && \\", end = '')
        # End of the recipes
        if len(self.recipes) > 0:
            print(":\n")
        else:
            print("\n")


class PhonyMakeRule:
    def __init__(self):
        self.target = ""
        self.dependencies = []

    def setTarget(self, target):
        self.target = target

    def addDependency(self, dependency):
        self.dependencies.append(dependency)

    def addDependencies(self, dependencies):
        self.dependencies.extend(dependencies)

    def print(self):
        # Print the target
        print(self.target, ": \\")
        # Print the dependencies
        for dependency in self.dependencies:
            print("\t" + dependency, "\\")
        # End of the dependencies
        print("\n")


def writeDefines():
    print("PROGRAM := ../build/bin/Release/LeafTipTriangulation.exe")
    print("PROGRAM_COMPARE := python3 ../Phenotyping/scripts/compare_to_ground_truth.py")
    print("PHENOTYPE := tips")
    print("\n")


def writeAllLeafCountingRuns(directories, thetas, probabilities, seeds):
    graphFiles = []

    for directory in directories:
        currentResultDirectory = directoryForResultFiles(directory)
        currentAnnotationDirectory = directoryContainingAnnotations(directory)
        currentGroundTruthFile = groundTruthFile(directory)

        currentResultAllFile = resultAllFile(directory)
        currentResultAllSortedFile = resultAllSortedFile(directory)

        allAggregateProbabilitiesAndSeedsResultAllFile = []

        for theta in thetas:

            allAggregateSeedsResultFile = []
            allAggregateSeedsResultBaseFile = []

            for probability in probabilities:
                
                allResultFiles = []
                allResultBaseFiles = []

                for seed in seeds:
                    currentResultFile = resultFile(directory, theta, probability, seed)
                    currentAnnotationFile = annotationFile(directory, theta, probability, seed)
                    currentResultBaseFile = resultBaseFile(directory, theta, probability, seed)

                    allResultFiles.append(currentResultFile)
                    allResultBaseFiles.append(currentResultBaseFile)

                    # Rule to run the leaf counting for all different parameters
                    rule = MakeRule()
                    rule.addTarget(currentResultFile)
                    rule.addTarget(currentAnnotationFile)
                    rule.addRecipe("mkdir -p {}".format(currentResultDirectory))
                    rule.addRecipe('$(PROGRAM) leaf_counting {} $(PHENOTYPE) {:d} {:d} {:.2f} {} {}'.format(currentAnnotationDirectory, theta, seed, probability, currentResultFile, currentAnnotationFile))
                    rule.print()

                    # Rule to convert the number of annotations to a baseline result CSV file
                    rule = MakeRule()
                    rule.addTarget(currentResultBaseFile)
                    rule.addDependency(currentAnnotationFile)
                    rule.addRecipe("{} {} > {}".format(extractBaselineFromAnnotationsCommand, currentAnnotationFile, currentResultBaseFile))
                    rule.print()

                aggregateSeedsResultFile = resultFileAggregateSeeds(directory, theta, probability)
                aggregateSeedsResultBaseFile = resultBaseFileAggregateSeeds(directory, theta, probability)

                allAggregateSeedsResultFile.append(aggregateSeedsResultFile)
                allAggregateSeedsResultBaseFile.append(aggregateSeedsResultBaseFile)

                # Rule to aggregate the results from all seeds
                rule = MakeRule()
                rule.addTarget(aggregateSeedsResultFile)
                rule.addDependencies(allResultFiles)
                rule.addRecipe('> {}'.format(aggregateSeedsResultFile)) # Clear the result file
                for currentResultFile in allResultFiles:
                    rule.addRecipe('$(PROGRAM_COMPARE) --command values --input {} --truth {} --output {} >> {}'.format(currentResultFile, currentGroundTruthFile, currentResultDirectory, aggregateSeedsResultFile))
                rule.print()

                # Rule to aggregate the baseline results from all seeds
                rule = MakeRule()
                rule.addTarget(aggregateSeedsResultBaseFile)
                rule.addDependencies(allResultBaseFiles)
                rule.addRecipe('> {}'.format(aggregateSeedsResultBaseFile)) # Clear the result file
                for currentResultBaseFile in allResultBaseFiles:
                    rule.addRecipe('$(PROGRAM_COMPARE) --command values --input {} --truth {} --output {} >> {}'.format(currentResultBaseFile, currentGroundTruthFile, currentResultDirectory, aggregateSeedsResultBaseFile))
                rule.print()

            aggregateProbabilitiesAndSeedsResultFile = resultFileAggregateProbabilitiesAndSeeds(directory, theta)
            aggregateProbabilitiesAndSeedsResultBaseFile = resultBaseFileAggregateProbabilitiesAndSeeds(directory, theta)
            aggregateProbabilitiesAndSeedsResultAllFile = resultAllFileAggregateProbabilitiesAndSeeds(directory, theta)

            allAggregateProbabilitiesAndSeedsResultAllFile.append(aggregateProbabilitiesAndSeedsResultAllFile)

            # Rule to aggregate the results from all probabilities
            rule = MakeRule()
            rule.addTarget(aggregateProbabilitiesAndSeedsResultFile)
            rule.addDependencies(allAggregateSeedsResultFile)
            rule.addRecipe('> {}'.format(aggregateProbabilitiesAndSeedsResultFile)) # Clear the result file
            for probability, aggregateSeedsResultFile in zip(probabilities, allAggregateSeedsResultFile):
                probabilityStr = integerProbabilityStr(probability)
                rule.addRecipe("printf '%s\t%s\t' {} {:d} >> {}".format(probabilityStr, theta, aggregateProbabilitiesAndSeedsResultFile))
                rule.addRecipe('datamash min 3 q1 3 median 3 q3 3 max 3 mean 3 sstdev 3 < {} >> {}'.format(aggregateSeedsResultFile, aggregateProbabilitiesAndSeedsResultFile))
            rule.print()

            # Rule to aggregate the baseline results from all probabilities
            rule = MakeRule()
            rule.addTarget(aggregateProbabilitiesAndSeedsResultBaseFile)
            rule.addDependencies(allAggregateSeedsResultBaseFile)
            rule.addRecipe('> {}'.format(aggregateProbabilitiesAndSeedsResultBaseFile)) # Clear the result file
            for probability, aggregateSeedsResultBaseFile in zip(probabilities, allAggregateSeedsResultBaseFile):
                probabilityStr = integerProbabilityStr(probability)
                rule.addRecipe('datamash min 3 q1 3 median 3 q3 3 max 3 mean 3 sstdev 3 < {} >> {}'.format(aggregateSeedsResultBaseFile, aggregateProbabilitiesAndSeedsResultBaseFile))
            rule.print()

            # Rule to merge the results and baseline into the same file
            rule = MakeRule()
            rule.addTarget(aggregateProbabilitiesAndSeedsResultAllFile)
            rule.addDependency(aggregateProbabilitiesAndSeedsResultFile)
            rule.addDependency(aggregateProbabilitiesAndSeedsResultBaseFile)
            rule.addRecipe('paste {} {} > {}'.format(aggregateProbabilitiesAndSeedsResultFile, aggregateProbabilitiesAndSeedsResultBaseFile, aggregateProbabilitiesAndSeedsResultAllFile))
            rule.print()

            agreementGraphFile = agreementGraph(directory, theta)
            graphFiles.append(agreementGraphFile)
            # Rule to generate a graph for the specific value of theta
            rule = MakeRule()
            rule.addTarget(agreementGraphFile)
            rule.addDependency(aggregateProbabilitiesAndSeedsResultAllFile)
            rule.addRecipe('gnuplot -e "filename=\'{}\'" "../plots/phenotyping_agreement.pg" > {}'.format(aggregateProbabilitiesAndSeedsResultAllFile, agreementGraphFile))
            rule.print()

        # Rule to merge all results for all theta values
        rule = MakeRule()
        rule.addTarget(currentResultAllFile)
        rule.addDependencies(allAggregateProbabilitiesAndSeedsResultAllFile)
        rule.addRecipe('> {}'.format(currentResultAllFile)) # Clear the result file
        for aggregateProbabilitiesAndSeedsResultAllFile in allAggregateProbabilitiesAndSeedsResultAllFile:
            rule.addRecipe('cat {} >> {}'.format(aggregateProbabilitiesAndSeedsResultAllFile, currentResultAllFile))
        rule.print()

        agreementThetaGraphFile = agreementThetaGraph(directory)
        agreementProbabilityGraphFile = agreementProbabilityGraph(directory)

        graphFiles.append(agreementThetaGraphFile)
        graphFiles.append(agreementProbabilityGraphFile)

        # Rule to generate graphs
        rule = MakeRule()
        rule.addTarget(agreementThetaGraphFile)
        rule.addDependency(currentResultAllFile)
        rule.addRecipe('gnuplot -e "filename=\'{}\'" "../plots/phenotyping_agreement_theta.pg" > {}'.format(currentResultAllFile, agreementThetaGraphFile))
        rule.print()

        rule = MakeRule()
        rule.addTarget(agreementProbabilityGraphFile)
        rule.addDependency(currentResultAllFile)
        rule.addRecipe('sort -n -k1,1 -k2,2 {} > {}'.format(currentResultAllFile, currentResultAllSortedFile))
        rule.addRecipe('gnuplot -e "filename=\'{}\'" "../plots/phenotyping_agreement_prob.pg" > {}'.format(currentResultAllSortedFile, agreementProbabilityGraphFile))
        rule.print()

    # Rule for all results
    rule = MakeRule()
    rule.addTarget('all')
    rule.addDependencies(graphFiles)
    rule.print()


def main():
    directories = ['sorghum_2018', 'sorghum_2022']
    thetas = [0, 500, 1000, 1500, 2000, 2500, 3000]
    probabilities = [0.0, 0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.45, 0.50]
    seeds = [14117, 4173, 6468, 306, 2456]

    writeDefines()
    writeAllLeafCountingRuns(directories, thetas, probabilities, seeds)


if __name__ == "__main__":
    main()
