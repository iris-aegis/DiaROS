<template>
    <v-row>
        <v-col cols=3 v-for="attr in dataset" :key="attr.name">
            <v-card
                class="mt-3 mx-auto text-xs-center"
                max-width="1500"
                color="white"
            >
                <v-sheet
                    class="mx-auto"
                    color="cyan"
                    max-width="calc(100% - 32px)"
                >
                    <v-sparkline
                        :labels="attr.label"
                        :value="attr.value"
                        color="white"
                        height="200"
                        line-width="2"
                    ></v-sparkline>
                </v-sheet>

                <v-card-text>
                    <div align="center" class="display-1 font-weight-thin">{{ attr.name }}</div>
                </v-card-text>

                <v-divider />

                <v-card-actions>
                    <v-spacer />
                    <v-btn outlined color="blue" @click="downloadCsv(attr.name)">CSV Download</v-btn>
                </v-card-actions>
            </v-card>
        </v-col>
    </v-row>
</template>

<script lang="ts">
import { Vue, Component, Watch } from 'vue-property-decorator';
import $ from 'jquery';

@Component({})
export default class ChartLine extends Vue {
    private value = [123, 233, 23, 342, 344, 344, 345, 332, 212, 23, 45, 34, 45, 67, 77];
    private attrinfo = { name: "TEST", label: this.value, value: this.value };
    private attrinfo1 = { name: "TEST1", label: this.value, value: this.value };
    private attrinfo2 = { name: "TEST2", label: this.value, value: this.value };
    private list = [ this.attrinfo, this.attrinfo1, this.attrinfo2 ];
    private dataset: { name: string, label: number[], value:number[] }[] = [];
    private eventId = 0;

    async created() {
        this.eventId = setInterval(this.update, 100);
    }

    private beforeDestroy() {
        clearInterval(this.eventId);
    }

    private async update(){
        const value = this.attrinfo2.value;
        value.shift();
        value.push( Math.floor( Math.random() * 301 ) );
        this.attrinfo2.label = value;
        this.attrinfo2.value = value;

        const result = $.ajax({
            url: '/getGraphData',
            type: 'GET',
            dataType: 'json'
        }).done((res:any) => {
            this.dataset = res.data;
        }).fail(() => {
            console.log("err");
        });
    }

    private downloadCsv( attr_name: string ) {
        $.ajax({
            url: '/getCsvData',
            type: 'GET',
            data: { name: attr_name },
            dataType: 'json'
        }).done((res:any) => {
            const csvData = res.data;
            const csvStr = this.generateCsvStr(attr_name, csvData);
            const blob = new Blob([csvStr], {type: 'text/csv'});
            const link = document.createElement('a');
            link.href = window.URL.createObjectURL(blob);
            link.download = `data_${attr_name}.csv`;
            link.click();
            console.log('download complete.');
        }).fail(() => {
            console.log("err");
        })
    }

    private generateCsvStr( attr_name: string, data: { target: number, time: number}[] ): string {
        let str = `\ufeff${attr_name},time\r\n`;
        data.map(obj => {
            str += `${obj.target},${obj.time}\r\n`;
        });
        return str;
    }
}

</script>