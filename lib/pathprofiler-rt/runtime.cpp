
#include <cstdint>
#include <cstdio>


extern "C" {


#define EPP(X)  PaThPrOfIlInG_ ## X

extern uint64_t EPP(paths)[];

extern uint64_t EPP(nPaths);

// arguments are supplied during instrumentation
void
EPP(count) (uint64_t funcid, uint64_t pathid) {
	if (funcid+pathid > EPP(nPaths))
	{
		printf("access bad path %llu\n", funcid+pathid);
	}
	EPP(paths)[funcid+pathid]++;
}

void
EPP(print) (void) {
	FILE* oFile;
	oFile = fopen("path-profile-results", "w");
	if (NULL != oFile) {
		for (uint64_t i = 0; i < EPP(nPaths); i++) {
			fprintf(oFile, "%llu\n", EPP(paths)[i]);
		}
		fclose(oFile);
	}
}


}

