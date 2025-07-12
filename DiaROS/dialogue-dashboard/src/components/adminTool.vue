<template>
    <v-data-table
        :headers="headers"
        :items="modTable"
        class="elevation-1"
        dark
    >
        <template v-slot:item.status="{item}">
            <td :class="getColor(item.status)">{{ item.status }}</td>
        </template>
    </v-data-table>
</template>

<script lang="ts">
import { Vue, Component, Watch } from 'vue-property-decorator';
import $ from 'jquery';

@Component({})
export default class AdminTool extends Vue {
    private eventId = 0;

    private modTable: { name: string, module: string, status: string }[] = [
        { name: "音声管理", module: "sm", status: "OFF" },
        { name: "言語理解", module: "lu", status: "OFF" },
        { name: "対話管理", module: "dm", status: "OFF" },
        { name: "応答制御", module: "rc", status: "OFF" },
        { name: "応答生成", module: "nlg", status: "OFF" },
        { name: "音声合成", module: "ss", status: "OFF" },
        { name: "データ受信", module: "dr", status: "OFF" }
    ];

    private headers = [
        { text: "Name", value: "name"},
        { text: "Module", value: "module"},
        { text: "Status", value: "status"},
    ];

    private getColor(status: string) {
        if(status == "ON") return "green--text";
        else return "red--text";
    }

    private created() {
        this.eventId = setInterval(this.update, 1000);
    }

    private beforeDestroy() {
        clearInterval(this.eventId);
    }

    private update() {
        const result = $.ajax({
            url: '/getModStatus',
            type: 'GET',
            dataType: 'json'
        }).done((res:any) => {
            const modStauts = res.data;
            for(const table of this.modTable) {
               const modName = table.module;
               table.status = modStauts[modName] ? "ON" : "OFF";
            }
        }).fail(() => {
            console.log("err");
        })
    }
}
</script>