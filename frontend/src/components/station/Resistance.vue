<template>
  <v-card>
    <v-card-title sm="6" class="grey darken-1 d-flex justify-center">
      <h5 class="white--text">
        Station de résistance
      </h5>
    </v-card-title>
    <v-row>
      <v-card-text>
        <h5 class="text-primary">{{ this.resistanceValue }} Ω</h5>
        <v-avatar
          size="30"
          v-for="(puckColor, i) in pucksColors"
          :key="i"
          :color="puckColor"
          >{{ i }}
        </v-avatar>
      </v-card-text>
    </v-row>
  </v-card>
</template>

<script lang="ts">
import { Component, Vue, Prop } from "vue-property-decorator";

@Component({
  components: {}
})
export default class Resistance extends Vue {
  @Prop() readonly resistanceValue!: number;
  @Prop() readonly pucksColors!:Array<string>;
}
</script>
