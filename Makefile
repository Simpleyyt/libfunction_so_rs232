projectName=function
sourceName=$(projectName).c
targeName=$(projectName).so

testName=test
testSource=$(testName).c
testTarge=$(testName)

$(testTarge):$(testSource) $(targeName)
	gcc $(testSource) -o $(testTarge) ./$(targeName)

$(targeName):$(sourceName)
	gcc -fPIC -shared $(sourceName) -o $(targeName)

clean:
	rm -f $(targeName)
	rm -f $(testTarge)

